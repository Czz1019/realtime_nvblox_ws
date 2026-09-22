from __future__ import annotations

from collections import deque
from pathlib import Path
import queue
import threading
import time
from typing import Callable

import numpy as np

from realtime_nvblox.cuvslam_backend import CuVslamBackend
from realtime_nvblox.latest_slot import LatestSlot
from realtime_nvblox.nvblox_backend import NvbloxBackend
from realtime_nvblox.pose_buffer import PoseBuffer
from realtime_nvblox.queues import DropOldestQueue
from realtime_nvblox.rate_limiter import RateLimiter
from realtime_nvblox.robot_pose import RobotPoseProvider
from realtime_nvblox.realsense_source import RealSenseSource, SourceQueues
from realtime_nvblox.stats import RuntimeStats
from realtime_nvblox.types import DepthFrame, Esdf3DRequest, EsdfData, MeshData, PoseSample


Callback = Callable[[object], None]


class RealtimeNvbloxRuntime:
    """RealSense + cuVSLAM + nvblox_torch realtime reconstruction runtime.

    Important architecture:
      - VIO worker owns cuVSLAM tracking.
      - Mapping worker does depth/color integration and ESDF/mesh updates.
      - 3D ESDF worker queries complete 3D fields independently.
      - 3D ESDF handoff is latest-only, so ROS/DDS cannot create an ever-growing
        backlog inside the mapper.
    """

    def __init__(self, cfg: dict):
        self.cfg = cfg
        rcfg = cfg['realsense']
        vcfg = cfg['vslam']
        mcfg = cfg['mapping']
        self.queues = SourceQueues(
            vio=DropOldestQueue(int(vcfg.get('vio_queue_size', 8))),
            depth=DropOldestQueue(int(mcfg.get('depth_queue_size', 4))),
            color=DropOldestQueue(int(mcfg.get('color_queue_size', 2))),
            imu=DropOldestQueue(int(vcfg.get('imu_queue_size', 2000))),
        )
        self.pose_source = cfg.get('pose_source', 'cuvslam')
        if self.pose_source not in ('cuvslam', 'robot'):
            raise ValueError('pose_source must be cuvslam or robot.')
        self.robot_pose = (RobotPoseProvider(cfg.get('robot_pose', {}), cfg['frames'])
                           if self.pose_source == 'robot' else None)
        self.source = RealSenseSource(rcfg, self.queues, robot_mode=self.robot_pose is not None)
        self.pose_buffer = PoseBuffer(int(vcfg.get('pose_buffer_size', 512)))
        self.vslam = None
        self.nvblox = None
        self.calibration = None
        self.stats = RuntimeStats()
        self.stop_event = threading.Event()
        self.threads: list[threading.Thread] = []
        self.error: Exception | None = None
        self.callbacks: dict[str, list[Callback]] = {
            'pose': [], 'mesh': [], 'esdf_slice': [], 'stats': []
        }
        self.latest_pose: PoseSample | None = None
        self._save_lock = threading.Lock()

        self.esdf3d_requests: LatestSlot[Esdf3DRequest] = LatestSlot()
        self.esdf3d_results: LatestSlot[EsdfData] = LatestSlot()
        self._esdf_version = 0
        self._latest_esdf_request: Esdf3DRequest | None = None

    def on(self, event: str, callback: Callback) -> None:
        if event not in self.callbacks:
            raise KeyError(event)
        self.callbacks[event].append(callback)

    def _emit(self, event: str, value) -> None:
        for cb in tuple(self.callbacks.get(event, ())):
            try:
                cb(value)
            except Exception:
                self.stats.inc(f'callback_error_{event}')

    def start(self) -> None:
        try:
            self.calibration = self.source.start()
            if self.robot_pose is not None:
                self.robot_pose.configure_calibration(self.calibration)
            elif bool(self.cfg['vslam'].get('enable', True)):
                self.vslam = CuVslamBackend(self.calibration, self.cfg['vslam'])
            else:
                raise RuntimeError('vslam.enable must be true for moving-camera reconstruction.')
            self.nvblox = NvbloxBackend(self.calibration, self.cfg['mapping'])

            if bool(self.cfg.get('esdf_3d', {}).get('enabled', True)):
                self.nvblox.prepare_esdf_grid(self.cfg['esdf_3d'])
        except Exception:
            self.source.stop()
            raise

        self.threads = [
            threading.Thread(target=self._mapping_loop_guarded, name='nvblox-mapping', daemon=True),
            threading.Thread(target=self._esdf3d_loop_guarded, name='nvblox-esdf3d', daemon=True),
        ]
        if self.robot_pose is None:
            self.threads.insert(0, threading.Thread(target=self._vio_loop_guarded, name='cuvslam-vio', daemon=True))
        for t in self.threads:
            t.start()

    def _set_error(self, exc: Exception) -> None:
        if self.error is None:
            self.error = exc
        self.stop_event.set()
        self.esdf3d_requests.close()
        self.esdf3d_results.close()

    def _vio_loop_guarded(self) -> None:
        try:
            self._vio_loop()
        except Exception as exc:
            self._set_error(exc)

    def _mapping_loop_guarded(self) -> None:
        try:
            self._mapping_loop()
        except Exception as exc:
            self._set_error(exc)

    def _esdf3d_loop_guarded(self) -> None:
        try:
            self._esdf3d_loop()
        except Exception as exc:
            self._set_error(exc)

    def _vio_loop(self) -> None:
        pending_imu = deque()
        # cuVSLAM has one strictly increasing timeline shared by IMU and images.
        last_tracker_ts = None
        imu_wait_s = float(self.cfg['vslam'].get('imu_wait_ms', 5.0)) / 1000.0

        while not self.stop_event.is_set():
            try:
                frame = self.queues.vio.get(timeout=0.2)
            except queue.Empty:
                continue

            frame_ts = int(frame.timestamp_ns)
            if last_tracker_ts is not None and frame_ts <= last_tracker_ts:
                self.stats.inc('vio_timestamp_drop')
                continue

            wait_deadline = time.monotonic() + imu_wait_s
            while True:
                drained = False
                try:
                    while True:
                        pending_imu.append(self.queues.imu.get_nowait())
                        drained = True
                except queue.Empty:
                    pass
                if pending_imu and pending_imu[-1].timestamp_ns >= frame_ts:
                    break
                if time.monotonic() >= wait_deadline:
                    break
                if not drained:
                    time.sleep(0.0005)

            while pending_imu and last_tracker_ts is not None and pending_imu[0].timestamp_ns <= last_tracker_ts:
                pending_imu.popleft()
                self.stats.inc('imu_timestamp_drop')

            # IMU at exactly frame_ts must not be registered before track(frame_ts).
            while pending_imu and pending_imu[0].timestamp_ns < frame_ts:
                sample = pending_imu.popleft()
                sample_ts = int(sample.timestamp_ns)
                if last_tracker_ts is not None and sample_ts <= last_tracker_ts:
                    self.stats.inc('imu_timestamp_drop')
                    continue
                self.vslam.register_imu(sample)
                last_tracker_ts = sample_ts
                self.stats.inc('imu_registered')

            if last_tracker_ts is not None and frame_ts <= last_tracker_ts:
                self.stats.inc('vio_timestamp_drop')
                continue

            start = time.perf_counter()
            pose = self.vslam.track(frame)
            self.stats.add_latency('vio_track', (time.perf_counter() - start) * 1000.0)
            last_tracker_ts = frame_ts

            if pose is None:
                self.stats.inc('vio_failed')
                continue

            self.pose_buffer.add(pose)
            self.latest_pose = pose
            self.stats.inc('vio_pose')
            self._emit('pose', pose)

    def add_robot_pose(self, timestamp_ns: int, T_base_tool: np.ndarray) -> bool:
        if self.robot_pose is None:
            raise RuntimeError('Robot pose input requires pose_source: robot.')
        accepted = self.robot_pose.add(timestamp_ns, T_base_tool)
        self.stats.inc('robot_pose_received' if accepted else 'robot_pose_out_of_order')
        return accepted

    def _pose_for(self, ts_ns: int) -> PoseSample | None:
        start = time.perf_counter()
        if self.robot_pose is not None:
            pose = self.robot_pose.query(ts_ns)
        else:
            vcfg = self.cfg['vslam']
            pose = self.pose_buffer.query(
                ts_ns,
                max_error_ms=float(vcfg.get('max_pose_error_ms', 60.0)),
                wait_ms=float(vcfg.get('pose_wait_ms', 35.0)),
            )
        self.stats.add_latency('pose_wait', (time.perf_counter() - start) * 1000.0)
        if pose is None or not pose.tracking_ok:
            return None
        self.stats.add_latency('pose_time_error', pose.pose_error_ms)
        self.stats.inc('pose_interpolated' if pose.interpolated else 'pose_exact_or_nearest')
        return pose

    @staticmethod
    def _grid_2d(center: np.ndarray, cfg: dict) -> np.ndarray:
        r = float(cfg['resolution_m'])
        xs = np.arange(center[0] - cfg['size_x_m'] / 2, center[0] + cfg['size_x_m'] / 2 + r * 0.5, r)
        ys = np.arange(center[1] - cfg['size_y_m'] / 2, center[1] + cfg['size_y_m'] / 2 + r * 0.5, r)
        xx, yy = np.meshgrid(xs, ys, indexing='xy')
        zz = np.full_like(xx, center[2], dtype=np.float32)
        return np.stack([xx, yy, zz], axis=-1).reshape(-1, 3).astype(np.float32)

    def _center_from_pose(self, pose: PoseSample, cfg: dict) -> np.ndarray:
        if bool(cfg.get('center_on_camera', True)):
            center = pose.T_world_rig[:3, 3].astype(np.float32).copy()
            offset = np.asarray([cfg.get('center_x_m', 0.0), cfg.get('center_y_m', 0.0),
                                 cfg.get('z_offset_m', 0.0)], dtype=np.float32)
            offset_frame = cfg.get('offset_frame', 'world')
            if offset_frame not in ('world', 'camera'):
                raise ValueError('esdf_3d.offset_frame must be world or camera.')
            if offset_frame == 'camera':
                offset = pose.T_world_rig[:3, :3] @ offset
            center += offset
            return center
        return np.asarray([
            cfg.get('center_x_m', 0.0),
            cfg.get('center_y_m', 0.0),
            cfg.get('center_z_m', 1.0),
        ], dtype=np.float32)

    def _mapping_loop(self) -> None:
        mcfg = self.cfg['mapping']
        depth_lim = RateLimiter(float(mcfg.get('depth_hz', 30.0)))
        color_lim = RateLimiter(float(mcfg.get('color_hz', 10.0)))
        esdf_lim = RateLimiter(float(mcfg.get('esdf_update_hz', 30.0)))
        mesh_lim = RateLimiter(float(mcfg.get('mesh_update_hz', 5.0)))
        mesh_out_lim = RateLimiter(float(mcfg.get('mesh_output_hz', 1.0)))
        slice_out_lim = RateLimiter(float(mcfg.get('esdf_slice_output_hz', 2.0)))
        stats_lim = RateLimiter(float(mcfg.get('stats_hz', 1.0)))
        dirty_pose = None
        dirty_timestamp_ns = 0

        while not self.stop_event.is_set():
            if self.source.error is not None:
                raise self.source.error
            timeout = min(0.1, esdf_lim.remaining()) if dirty_pose is not None else 0.1
            try:
                depth: DepthFrame = self.queues.depth.get(timeout=timeout)
            except queue.Empty:
                depth = None

            integrated = False
            depth_pose = None
            if depth is not None and depth_lim.ready():
                depth_pose = self._pose_for(depth.timestamp_ns)
                if depth_pose is None:
                    self.stats.inc('depth_pose_drop')
                else:
                    T_world_depth = depth_pose.T_world_rig @ self.calibration.T_rig_depth
                    start = time.perf_counter()
                    self.nvblox.integrate_depth(depth.depth_raw, T_world_depth)
                    self.stats.add_latency('depth_integrate', (time.perf_counter() - start) * 1000.0)
                    self.stats.inc('depth_integrated')
                    integrated = True
                    dirty_pose = depth_pose
                    dirty_timestamp_ns = depth.timestamp_ns
                    if self.robot_pose is not None:
                        self.stats.add_latency('depth_age', (time.time_ns() - depth.timestamp_ns) / 1e6)
                        self.latest_pose = depth_pose
                        self._emit('pose', depth_pose)
            elif depth is not None:
                self.stats.inc('depth_rate_skipped')

            try:
                color = self.queues.color.get_nowait()
            except queue.Empty:
                color = None
            if color is not None and bool(mcfg.get('enable_color', True)) and color_lim.ready():
                pose = self._pose_for(color.timestamp_ns)
                if pose is None:
                    self.stats.inc('color_pose_drop')
                else:
                    T_world_color = pose.T_world_rig @ self.calibration.T_rig_color
                    start = time.perf_counter()
                    self.nvblox.integrate_color(color.color_rgb, T_world_color)
                    self.stats.add_latency('color_integrate', (time.perf_counter() - start) * 1000.0)
                    self.stats.inc('color_integrated')

            now = time.monotonic()
            if dirty_pose is not None and bool(mcfg.get('enable_esdf', True)) and esdf_lim.ready(now):
                start = time.perf_counter()
                # Metadata and ESDF layer advance under the same mapper lock.
                with self.nvblox.lock:
                    self.nvblox.update_esdf()
                    self._esdf_version += 1
                    ecfg = self.cfg['esdf_3d']
                    self._latest_esdf_request = Esdf3DRequest(
                        timestamp_ns=int(dirty_timestamp_ns),
                        center_xyz=self._center_from_pose(dirty_pose, ecfg),
                        version=self._esdf_version,
                    )
                    if bool(ecfg.get('enabled', True)):
                        self.esdf3d_requests.put(self._latest_esdf_request)
                        self.stats.inc('esdf_3d_requested')
                self.stats.add_latency('esdf_update', (time.perf_counter() - start) * 1000.0)
                self.stats.inc('esdf_updated')
                if slice_out_lim.ready(now):
                    self._output_esdf_slice(dirty_timestamp_ns)
                dirty_pose = None
            elif dirty_pose is not None and not bool(mcfg.get('enable_esdf', True)):
                dirty_pose = None

            if integrated and bool(mcfg.get('enable_mesh', True)) and mesh_lim.ready(now):
                start = time.perf_counter()
                self.nvblox.update_mesh()
                self.stats.add_latency('mesh_update', (time.perf_counter() - start) * 1000.0)
                self.stats.inc('mesh_updated')
                if mesh_out_lim.ready(now):
                    start = time.perf_counter()
                    v, t, c = self.nvblox.mesh_numpy()
                    self.stats.add_latency('mesh_to_cpu', (time.perf_counter() - start) * 1000.0)
                    self._emit('mesh', MeshData(depth.timestamp_ns, v, t, c))
                    self.stats.inc('mesh_output')

            if stats_lim.ready(now):
                self._publish_stats()

    def _esdf3d_loop(self) -> None:
        cfg = self.cfg['esdf_3d']
        if not bool(cfg.get('enabled', True)):
            return
        hz = float(cfg.get('generate_hz', self.cfg['mapping'].get('esdf_3d_output_hz', 30.0)))
        limiter = RateLimiter(hz)
        last_sequence = 0
        last_version = 0

        while not self.stop_event.is_set():
            sequence, req = self.esdf3d_requests.wait_newer(last_sequence, timeout=0.2)
            if req is None:
                continue
            # Keep pending work until its deadline, then sample the newest request.
            if self.stop_event.wait(limiter.remaining()):
                break
            if not limiter.ready():
                continue
            start = time.perf_counter()
            with self.nvblox.lock:
                self.stats.add_latency('esdf_3d_lock_wait', (time.perf_counter() - start) * 1000.0)
                sequence, _ = self.esdf3d_requests.latest()
                req = self._latest_esdf_request
                if sequence > last_sequence + 1:
                    self.stats.inc('esdf_3d_request_skipped', sequence - last_sequence - 1)
                last_sequence = sequence
                if req is None or req.version <= last_version:
                    continue
                pts, dist = self.nvblox.query_esdf_grid(req.center_xyz, cfg)
                last_version = req.version
                for name, ms in self.nvblox.query_timings_ms.items():
                    self.stats.add_latency(name, ms)
            self.stats.add_latency('esdf_3d_query', (time.perf_counter() - start) * 1000.0)
            self.esdf3d_results.put(EsdfData(req.timestamp_ns, pts, dist, version=req.version))
            self.stats.inc('esdf_3d_generated')

    def _query_center(self, fallback_z: float = 0.0) -> np.ndarray:
        if self.latest_pose is None:
            return np.array([0.0, 0.0, fallback_z], dtype=np.float32)
        return self.latest_pose.T_world_rig[:3, 3].astype(np.float32).copy()

    def _output_esdf_slice(self, ts: int) -> None:
        cfg = self.cfg['esdf_slice']
        if not bool(cfg.get('enabled', True)):
            return
        center = self._query_center(float(cfg.get('z_m', 0.5)))
        if str(cfg.get('z_mode', 'camera_relative')) == 'camera_relative':
            center[2] += float(cfg.get('z_offset_m', 0.0))
        else:
            center[2] = float(cfg.get('z_m', 0.5))
        pts = self._grid_2d(center, cfg)
        start = time.perf_counter()
        d = self.nvblox.query_esdf(pts)
        self.stats.add_latency('esdf_slice_query', (time.perf_counter() - start) * 1000.0)
        self._emit('esdf_slice', EsdfData(ts, pts, d))
        self.stats.inc('esdf_slice_output')

    def _extra_stats(self) -> dict:
        esdf_seq, latest = self.esdf3d_results.latest()
        return {
            'queues': {
                'vio': self.queues.vio.qsize(), 'depth': self.queues.depth.qsize(),
                'color': self.queues.color.qsize(), 'imu': self.queues.imu.qsize(),
            },
            'queue_drops': {
                'vio': self.queues.vio.dropped, 'depth': self.queues.depth.dropped,
                'color': self.queues.color.dropped, 'imu': self.queues.imu.dropped,
            },
            'esdf_3d': {
                'latest_sequence': int(esdf_seq),
                'latest_version': int(latest.version) if latest is not None else 0,
                'num_points': int(len(latest.distance_m)) if latest is not None else 0,
            },
            'realsense': dict(self.source.stats),
            'pose_source': self.pose_source,
            'vslam': {
                'tracked_frames': getattr(self.vslam, 'tracked_frames', 0),
                'failed_frames': getattr(self.vslam, 'failed_frames', 0),
                'registered_imu': getattr(self.vslam, 'registered_imu', 0),
            },
            'nvblox_library': str(getattr(self.nvblox, 'loaded_lib', '')),
        }

    def _publish_stats(self) -> None:
        self._emit('stats', self.stats.snapshot(self._extra_stats()))

    def save_map(self) -> Path:
        with self._save_lock:
            out = Path(self.cfg['output']['directory']).expanduser()
            return self.nvblox.save_map(out / self.cfg['output']['map_filename'])

    def save_mesh(self) -> Path:
        with self._save_lock:
            out = Path(self.cfg['output']['directory']).expanduser()
            return self.nvblox.save_mesh(out / self.cfg['output']['mesh_filename'])

    def stop(self) -> None:
        if self.stop_event.is_set() and self.nvblox is None:
            return
        self.stop_event.set()
        self.esdf3d_requests.close()
        self.esdf3d_results.close()
        self.source.stop()
        for t in self.threads:
            if t.is_alive():
                t.join(timeout=2.0)
        if self.nvblox is not None:
            out_cfg = self.cfg['output']
            try:
                if bool(out_cfg.get('save_map_on_exit', False)):
                    self.save_map()
                if bool(out_cfg.get('save_mesh_on_exit', True)):
                    self.save_mesh()
                out = Path(out_cfg['directory']).expanduser()
                self.stats.write_json(out / out_cfg.get('stats_json', 'runtime_stats.json'), self._extra_stats())
            except Exception:
                pass

    def run_forever(self) -> None:
        self.start()
        try:
            while not self.stop_event.is_set():
                if self.error is not None:
                    raise RuntimeError(f'Runtime worker failed: {self.error!r}')
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
