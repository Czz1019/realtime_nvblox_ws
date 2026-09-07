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
    """Load the nvblox shared library bundled with the nvblox_torch wheel."""
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
            import torch  # load c10/libtorch first
            ctypes.CDLL(str(lib), mode=ctypes.RTLD_GLOBAL)
            _PRELOADED = True
        return lib


class NvbloxBackend:
    """Thread-safe wrapper around one nvblox_torch Mapper.

    Mapper access is serialized by one RLock.  The runtime may therefore keep
    mapping and 3D-ESDF query workers separate without allowing two Python
    threads to mutate/query the same mapper object simultaneously.
    """

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
        self.esdf_unknown_distance = 1.0e6

        self._grid_key = None
        self._grid_offsets_cpu = None
        self._grid_offsets_gpu = None
        self._grid_query_gpu = None
        self._grid_output_gpu = None

        self._initialize()

    def _initialize(self) -> None:
        self.loaded_lib = preload_nvblox_runtime()
        import torch
        from nvblox_torch.mapper import Mapper, QueryType
        from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType
        from nvblox_torch.sensor import Sensor
        try:
            from nvblox_torch.constants import constants
            self.esdf_unknown_distance = float(constants.esdf_unknown_distance())
        except Exception:
            self.esdf_unknown_distance = 1.0e6

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
            valid = self.torch.isfinite(depth) & (depth > 0.0)
            max_d = float(self.cfg.get('max_integration_distance_m', 0.0))
            if max_d > 0.0:
                valid = valid & (depth <= max_d)
            # Avoid invalid.any().item(), which introduces a CPU/GPU sync every frame.
            depth = self.torch.where(valid, depth, self.torch.zeros((), device='cuda', dtype=self.torch.float32))
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
        """Prepare Nx4 [x,y,z,radius] query for the installed v0.0.10 binding."""
        pts = np.asarray(points_xyz, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] not in (3, 4):
            raise ValueError(f'ESDF query must have shape (N,3) or (N,4), got {pts.shape}.')
        q = self.torch.as_tensor(pts, device='cuda', dtype=self.torch.float32).contiguous()
        if q.shape[1] == 3:
            radius = self.torch.zeros((q.shape[0], 1), device=q.device, dtype=q.dtype)
            q = self.torch.cat((q, radius), dim=1).contiguous()
        return q

    def query_esdf(self, points_xyz: np.ndarray) -> np.ndarray:
        with self.lock:
            q = self._prepare_esdf_query(points_xyz)
            out = self.mapper.query_layer(self.QueryType.ESDF, q, mapper_id=0)
            owned = out.detach().clone()
        return owned.cpu().numpy().reshape(-1)

    def query_esdf_with_gradients(self, points_xyz: np.ndarray) -> np.ndarray:
        with self.lock:
            q = self._prepare_esdf_query(points_xyz)
            out = self.mapper.query_layer(self.QueryType.ESDF_GRAD, q, mapper_id=0)
            owned = out.detach().clone()
        return owned.cpu().numpy()

    @staticmethod
    def _grid_key_from_cfg(cfg: dict):
        return (
            float(cfg['resolution_m']),
            float(cfg['size_x_m']),
            float(cfg['size_y_m']),
            float(cfg['size_z_m']),
        )

    def prepare_esdf_grid(self, cfg: dict) -> None:
        key = self._grid_key_from_cfg(cfg)
        if self._grid_key == key:
            return
        r, sx, sy, sz = key
        # Keep the same inclusive-end convention used by the original runtime.
        xs = np.arange(-sx / 2.0, sx / 2.0 + r * 0.5, r, dtype=np.float32)
        ys = np.arange(-sy / 2.0, sy / 2.0 + r * 0.5, r, dtype=np.float32)
        zs = np.arange(-sz / 2.0, sz / 2.0 + r * 0.5, r, dtype=np.float32)
        xx, yy, zz = np.meshgrid(xs, ys, zs, indexing='xy')
        offsets = np.stack((xx, yy, zz), axis=-1).reshape(-1, 3).astype(np.float32, copy=False)

        with self.lock:
            offsets_gpu = self.torch.as_tensor(offsets, device='cuda', dtype=self.torch.float32).contiguous()
            query_gpu = self.torch.empty((offsets_gpu.shape[0], 4), device='cuda', dtype=self.torch.float32)
            query_gpu[:, 3].zero_()
            output_gpu = self.torch.empty((offsets_gpu.shape[0], 1), device='cuda', dtype=self.torch.float32)
            self._grid_key = key
            self._grid_offsets_cpu = offsets
            self._grid_offsets_gpu = offsets_gpu
            self._grid_query_gpu = query_gpu
            self._grid_output_gpu = output_gpu

    def query_esdf_grid(self, center_xyz: np.ndarray, cfg: dict) -> tuple[np.ndarray, np.ndarray]:
        """Query a complete local 3D ESDF grid using preallocated GPU buffers."""
        self.prepare_esdf_grid(cfg)
        center = np.asarray(center_xyz, dtype=np.float32).reshape(3)
        center_cpu = center.copy()

        with self.lock:
            center_gpu = self.torch.as_tensor(center, device='cuda', dtype=self.torch.float32)
            self._grid_query_gpu[:, :3].copy_(self._grid_offsets_gpu + center_gpu)
            self._grid_output_gpu.fill_(float(self.esdf_unknown_distance))
            out = self.mapper.query_layer(
                self.QueryType.ESDF,
                self._grid_query_gpu,
                output=self._grid_output_gpu,
                mapper_id=0,
            )
            # Own the result so the preallocated output can be reused immediately.
            owned = out.detach().clone()

        distances = owned.cpu().numpy().reshape(-1)
        points = self._grid_offsets_cpu + center_cpu.reshape(1, 3)
        return points.astype(np.float32, copy=False), distances.astype(np.float32, copy=False)

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
