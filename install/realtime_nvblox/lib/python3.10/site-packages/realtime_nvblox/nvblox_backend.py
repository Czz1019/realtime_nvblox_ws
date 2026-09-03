from __future__ import annotations

import ctypes
import importlib.util
from pathlib import Path
import threading

import numpy as np

from realtime_nvblox.types import Calibration


_PRELOAD_LOCK = threading.Lock()
_PRELOADED = False


def preload_nvblox_runtime() -> Path:
    """Load wheel-bundled libnvblox_lib.so before importing libpy_nvblox.so.

    This makes the Python package self-contained even when the shell happens to
    contain a different ROS/C++ nvblox library on LD_LIBRARY_PATH.
    """
    global _PRELOADED
    with _PRELOAD_LOCK:
        spec = importlib.util.find_spec('nvblox_torch')
        if spec is None or not spec.submodule_search_locations:
            raise ImportError('nvblox_torch is not installed.')
        pkg = Path(next(iter(spec.submodule_search_locations)))
        lib = pkg / 'lib' / 'nvblox' / 'libnvblox_lib.so'
        if not lib.exists():
            raise FileNotFoundError(f'nvblox_torch bundled library not found: {lib}')
        if not _PRELOADED:
            import torch  # loads libtorch/c10 first
            ctypes.CDLL(str(lib), mode=ctypes.RTLD_GLOBAL)
            _PRELOADED = True
        return lib


class NvbloxBackend:
    def __init__(self, calibration: Calibration, cfg: dict):
        self.calib = calibration
        self.cfg = cfg
        self.lock = threading.RLock()
        self.mapper = None
        self.torch = None
        self.QueryType = None
        self.depth_sensor = None
        self.color_sensor = None
        self.loaded_lib = None
        self._initialize()

    def _initialize(self) -> None:
        self.loaded_lib = preload_nvblox_runtime()
        import torch
        from nvblox_torch.mapper import Mapper, QueryType
        from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType
        from nvblox_torch.sensor import Sensor

        if not torch.cuda.is_available():
            raise RuntimeError('CUDA is not available to PyTorch.')
        self.torch = torch
        self.QueryType = QueryType
        d = self.calib.depth_intrinsics
        c = self.calib.color_intrinsics
        self.depth_sensor = Sensor.from_camera(d.fx, d.fy, d.cx, d.cy, d.width, d.height)
        self.color_sensor = Sensor.from_camera(c.fx, c.fy, c.cx, c.cy, c.width, c.height)
        self.mapper = Mapper(
            voxel_sizes_m=float(self.cfg.get('voxel_size_m', 0.05)),
            integrator_types=ProjectiveIntegratorType.TSDF,
        )

    def _pose(self, T: np.ndarray):
        return self.torch.from_numpy(np.asarray(T, dtype=np.float32)).contiguous().cpu()

    def integrate_depth(self, depth_raw: np.ndarray, T_world_depth: np.ndarray) -> None:
        with self.lock:
            depth = self.torch.as_tensor(depth_raw, device='cuda', dtype=self.torch.float32)
            depth = depth * float(self.calib.depth_scale_m)
            invalid = (~self.torch.isfinite(depth)) | (depth <= 0.0)
            max_d = float(self.cfg.get('max_integration_distance_m', 0.0))
            if max_d > 0:
                invalid |= depth > max_d
            if bool(invalid.any()):
                depth = depth.clone()
                depth[invalid] = 0.0
            self.mapper.add_depth_frame(depth.contiguous(), self._pose(T_world_depth), self.depth_sensor)

    def integrate_color(self, color_rgb: np.ndarray, T_world_color: np.ndarray) -> None:
        with self.lock:
            color = self.torch.as_tensor(color_rgb, device='cuda', dtype=self.torch.uint8).contiguous()
            self.mapper.add_color_frame(color, self._pose(T_world_color), self.color_sensor)

    def update_esdf(self) -> None:
        with self.lock:
            self.mapper.update_esdf(0)

    def update_mesh(self) -> None:
        with self.lock:
            self.mapper.update_color_mesh(0)

    def mesh_numpy(self):
        with self.lock:
            mesh = self.mapper.get_color_mesh(0)
            return (
                mesh.vertices().detach().cpu().numpy(),
                mesh.triangles().detach().cpu().numpy(),
                mesh.vertex_colors().detach().cpu().numpy(),
            )

    def _prepare_esdf_query(self, points_xyz: np.ndarray):
        """Prepare an ESDF query tensor for nvblox_torch v0.0.10.

        The public Python API documents Nx3 XYZ query points (optionally Nx4
        with a sphere radius), but the v0.0.10 C++ binding requires Nx4 for
        queryEsdf(). Use a zero radius for ordinary point-distance queries.
        """
        pts = np.asarray(points_xyz, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] not in (3, 4):
            raise ValueError(
                f'ESDF query must have shape (N,3) or (N,4), got {pts.shape}.'
            )

        q = self.torch.as_tensor(
            pts, device='cuda', dtype=self.torch.float32
        ).contiguous()

        if q.shape[1] == 3:
            radius = self.torch.zeros(
                (q.shape[0], 1),
                device=q.device,
                dtype=q.dtype,
            )
            q = self.torch.cat((q, radius), dim=1).contiguous()

        return q

    def query_esdf(self, points_xyz: np.ndarray) -> np.ndarray:
        with self.lock:
            q = self._prepare_esdf_query(points_xyz)
            out = self.mapper.query_layer(
                self.QueryType.ESDF, q, mapper_id=0
            )
            return out.detach().cpu().numpy().reshape(-1)

    def query_esdf_with_gradients(self, points_xyz: np.ndarray) -> np.ndarray:
        with self.lock:
            q = self._prepare_esdf_query(points_xyz)
            out = self.mapper.query_layer(
                self.QueryType.ESDF_GRAD, q, mapper_id=0
            )
            return out.detach().cpu().numpy()

    def save_map(self, path: str | Path) -> Path:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        with self.lock:
            self.mapper.save_map(str(path), 0)
        return path

    def save_mesh(self, path: str | Path) -> Path:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        with self.lock:
            self.mapper.update_color_mesh(0)
            self.mapper.get_color_mesh(0).save(str(path))
        return path
