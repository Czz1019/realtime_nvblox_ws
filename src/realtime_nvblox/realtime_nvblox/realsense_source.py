from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pyrealsense2 as rs

from realtime_nvblox.queues import DropOldestQueue
from realtime_nvblox.types import (
    Calibration,
    CameraIntrinsics,
    ColorFrame,
    DepthFrame,
    ImuSample,
    VioFrame,
)


def _intrinsics(profile) -> CameraIntrinsics:
    intr = profile.as_video_stream_profile().get_intrinsics()
    return CameraIntrinsics(
        width=int(intr.width), height=int(intr.height),
        fx=float(intr.fx), fy=float(intr.fy),
        cx=float(intr.ppx), cy=float(intr.ppy),
    )


def _extrinsics_matrix(source_profile, target_profile) -> np.ndarray:
    """Return T_target_source from librealsense source.get_extrinsics_to(target)."""
    ext = source_profile.get_extrinsics_to(target_profile)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = np.asarray(ext.rotation, dtype=np.float32).reshape(3, 3)
    T[:3, 3] = np.asarray(ext.translation, dtype=np.float32)
    return T


def _device_timestamp_ns(frame) -> int:
    return int(round(float(frame.get_timestamp()) * 1e6))


class EmitterClassifier:
    """Python equivalent of the RealSense splitter's emitter-state selection.

    Prefer exact per-frame metadata. If firmware does not expose it, a small
    calibration estimates which frame-number parity has the higher valid-depth
    ratio and treats that parity as emitter-on.
    """

    def __init__(self, calibration_frames: int = 20, allow_fallback: bool = True):
        self.calibration_frames = max(4, int(calibration_frames))
        self.allow_fallback = allow_fallback
        self._parity_scores = {0: [], 1: []}
        self._on_parity: Optional[int] = None
        self.using_metadata = False

    def is_on(self, depth_frame, depth_array: np.ndarray) -> Optional[bool]:
        metadata_candidates = []
        if hasattr(rs.frame_metadata_value, 'frame_emitter_mode'):
            metadata_candidates.append(rs.frame_metadata_value.frame_emitter_mode)
        if hasattr(rs.frame_metadata_value, 'frame_laser_power_mode'):
            metadata_candidates.append(rs.frame_metadata_value.frame_laser_power_mode)

        for key in metadata_candidates:
            try:
                if depth_frame.supports_frame_metadata(key):
                    self.using_metadata = True
                    return int(depth_frame.get_frame_metadata(key)) != 0
            except Exception:
                pass

        if not self.allow_fallback:
            return None

        parity = int(depth_frame.get_frame_number()) & 1
        if self._on_parity is None:
            valid_ratio = float(np.count_nonzero(depth_array)) / max(1, depth_array.size)
            self._parity_scores[parity].append(valid_ratio)
            count = len(self._parity_scores[0]) + len(self._parity_scores[1])
            if count >= self.calibration_frames and self._parity_scores[0] and self._parity_scores[1]:
                m0 = float(np.mean(self._parity_scores[0]))
                m1 = float(np.mean(self._parity_scores[1]))
                self._on_parity = 0 if m0 >= m1 else 1
            else:
                return None
        return parity == self._on_parity


@dataclass
class SourceQueues:
    vio: DropOldestQueue[VioFrame]
    depth: DropOldestQueue[DepthFrame]
    color: DropOldestQueue[ColorFrame]
    imu: DropOldestQueue[ImuSample]


class RealSenseSource:
    def __init__(self, cfg: dict, queues: SourceQueues):
        self.cfg = cfg
        self.queues = queues
        self.stop_event = threading.Event()
        self.video_pipe = rs.pipeline()
        self.color_pipe = rs.pipeline()
        self.motion_pipe = rs.pipeline()
        self.video_profile = None
        self.color_profile = None
        self.motion_profile = None
        self.calibration: Optional[Calibration] = None
        self.threads: list[threading.Thread] = []
        self._latest_accel: Optional[np.ndarray] = None
        self._latest_accel_ts: Optional[int] = None
        self._motion_lock = threading.Lock()
        self._emitter = EmitterClassifier(
            int(cfg.get('parity_calibration_frames', 20)),
            allow_fallback=not bool(cfg.get('emitter_metadata_required', False)),
        )
        self.stats = {
            'video_frames': 0, 'depth_frames': 0, 'vio_frames': 0,
            'color_frames': 0, 'imu_frames': 0, 'unknown_emitter_frames': 0,
        }

    def _serial(self) -> str:
        requested = str(self.cfg.get('serial', '')).strip()
        ctx = rs.context()
        devices = list(ctx.query_devices())
        if not devices:
            raise RuntimeError('No RealSense device found.')
        if requested:
            for dev in devices:
                if dev.get_info(rs.camera_info.serial_number) == requested:
                    return requested
            raise RuntimeError(f'RealSense serial {requested} not found.')
        return devices[0].get_info(rs.camera_info.serial_number)

    @staticmethod
    def _nearest_motion_fps(device, stream_type, preferred: int) -> int:
        values = []
        for sensor in device.query_sensors():
            for profile in sensor.get_stream_profiles():
                try:
                    if profile.stream_type() == stream_type:
                        values.append(int(profile.fps()))
                except Exception:
                    pass
        values = sorted(set(x for x in values if x > 0))
        if not values:
            raise RuntimeError(f'No supported RealSense profile for {stream_type}.')
        return min(values, key=lambda x: abs(x - int(preferred)))

    def _calibrate_once(
        self,
        serial: str,
        dw: int,
        dh: int,
        dfps: int,
        cw: int,
        ch: int,
        cfps: int,
        accel_fps: int,
        gyro_fps: int,
    ) -> Calibration:
        """Read all intrinsics/extrinsics from one temporary pipeline.

        librealsense extrinsics are static device calibration, but querying
        get_extrinsics_to() across stream profiles that came from different
        rs.pipeline instances can fail with "Requested extrinsics are not
        available".  We therefore activate every required stream once in a
        single pipeline, read calibration, stop it, and afterwards run the
        high-rate video, color and IMU sensors in independent pipelines.
        """
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)
        cfg.enable_stream(rs.stream.infrared, 1, dw, dh, rs.format.y8, dfps)
        cfg.enable_stream(rs.stream.infrared, 2, dw, dh, rs.format.y8, dfps)
        cfg.enable_stream(rs.stream.color, cw, ch, rs.format.rgb8, cfps)
        cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, accel_fps)
        cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, gyro_fps)

        profile = None
        try:
            profile = pipe.start(cfg)
            # Give all sensors a moment to become active before asking the
            # librealsense calibration graph for cross-sensor extrinsics.
            time.sleep(0.15)

            left_p = profile.get_stream(rs.stream.infrared, 1)
            right_p = profile.get_stream(rs.stream.infrared, 2)
            depth_p = profile.get_stream(rs.stream.depth)
            color_p = profile.get_stream(rs.stream.color)
            accel_p = profile.get_stream(rs.stream.accel)
            depth_sensor = profile.get_device().first_depth_sensor()

            return Calibration(
                left_intrinsics=_intrinsics(left_p),
                right_intrinsics=_intrinsics(right_p),
                depth_intrinsics=_intrinsics(depth_p),
                color_intrinsics=_intrinsics(color_p),
                T_rig_right=_extrinsics_matrix(right_p, left_p),
                T_rig_depth=_extrinsics_matrix(depth_p, left_p),
                T_rig_color=_extrinsics_matrix(color_p, left_p),
                T_rig_imu=_extrinsics_matrix(accel_p, left_p),
                depth_scale_m=float(depth_sensor.get_depth_scale()),
                imu_frequency_hz=float(gyro_fps),
            )
        finally:
            if profile is not None:
                try:
                    pipe.stop()
                except Exception:
                    pass
            # Some D4xx firmware/USB stacks need a short release interval
            # before the same physical sensors are opened by new pipelines.
            time.sleep(0.20)

    def start(self) -> Calibration:
        serial = self._serial()
        dw = int(self.cfg.get('depth_width', 640))
        dh = int(self.cfg.get('depth_height', 480))
        dfps = int(self.cfg.get('depth_fps', 60))
        cw = int(self.cfg.get('color_width', 640))
        ch = int(self.cfg.get('color_height', 480))
        cfps = int(self.cfg.get('color_fps', 15))

        # Resolve motion rates before any long-running pipeline is opened.
        ctx = rs.context()
        device = None
        for dev in ctx.query_devices():
            if dev.get_info(rs.camera_info.serial_number) == serial:
                device = dev
                break
        if device is None:
            raise RuntimeError(f'RealSense serial {serial} disappeared before startup.')

        accel_fps = self._nearest_motion_fps(
            device, rs.stream.accel, int(self.cfg.get('preferred_accel_fps', 200))
        )
        gyro_fps = self._nearest_motion_fps(
            device, rs.stream.gyro, int(self.cfg.get('preferred_gyro_fps', 200))
        )

        try:
            # IMPORTANT: obtain all static calibration from stream profiles
            # belonging to ONE pipeline.  Do not query cross-pipeline profiles.
            self.calibration = self._calibrate_once(
                serial, dw, dh, dfps, cw, ch, cfps, accel_fps, gyro_fps
            )

            # Depth/IR share the stereo module and must remain synchronized.
            video_cfg = rs.config()
            video_cfg.enable_device(serial)
            video_cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)
            video_cfg.enable_stream(rs.stream.infrared, 1, dw, dh, rs.format.y8, dfps)
            video_cfg.enable_stream(rs.stream.infrared, 2, dw, dh, rs.format.y8, dfps)
            self.video_profile = self.video_pipe.start(video_cfg)

            depth_sensor = self.video_profile.get_device().first_depth_sensor()
            if depth_sensor.supports(rs.option.frames_queue_size):
                depth_sensor.set_option(
                    rs.option.frames_queue_size,
                    float(self.cfg.get('sensor_queue_size', 2)),
                )
            if bool(self.cfg.get('emitter_flashing', True)):
                if depth_sensor.supports(rs.option.emitter_enabled):
                    depth_sensor.set_option(rs.option.emitter_enabled, 1.0)
                if not depth_sensor.supports(rs.option.emitter_on_off):
                    raise RuntimeError(
                        'This RealSense depth sensor does not support emitter_on_off.'
                    )
                depth_sensor.set_option(rs.option.emitter_on_off, 1.0)

            # Keep color independent so 15 Hz color cannot throttle the 60 Hz
            # stereo/depth sensor pipeline.
            color_cfg = rs.config()
            color_cfg.enable_device(serial)
            color_cfg.enable_stream(rs.stream.color, cw, ch, rs.format.rgb8, cfps)
            self.color_profile = self.color_pipe.start(color_cfg)

            # Keep motion independent as well; callback pushes bounded IMU data.
            motion_cfg = rs.config()
            motion_cfg.enable_device(serial)
            motion_cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, accel_fps)
            motion_cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, gyro_fps)
            self.motion_profile = self.motion_pipe.start(motion_cfg, self._motion_callback)

            self.threads = [
                threading.Thread(target=self._video_loop, name='rs-depth-ir', daemon=True),
                threading.Thread(target=self._color_loop, name='rs-color', daemon=True),
            ]
            for t in self.threads:
                t.start()
            return self.calibration
        except Exception:
            # Do not leave a camera sensor busy after a partially failed start.
            self.stop()
            raise

    def _motion_callback(self, frame) -> None:
        if self.stop_event.is_set():
            return
        frames = []
        try:
            if frame.is_frameset():
                frames = list(frame.as_frameset())
            else:
                frames = [frame]
        except Exception:
            frames = [frame]
        for f in frames:
            try:
                st = f.get_profile().stream_type()
                if st not in (rs.stream.accel, rs.stream.gyro):
                    continue
                data = f.as_motion_frame().get_motion_data()
                vec = np.array([data.x, data.y, data.z], dtype=np.float64)
                ts = _device_timestamp_ns(f)
                with self._motion_lock:
                    if st == rs.stream.accel:
                        self._latest_accel = vec
                        self._latest_accel_ts = ts
                    elif self._latest_accel is not None:
                        self.queues.imu.put(ImuSample(ts, self._latest_accel.copy(), vec))
                        self.stats['imu_frames'] += 1
            except Exception:
                continue

    def _video_loop(self) -> None:
        warmup = int(self.cfg.get('warmup_frames', 30))
        seen = 0
        while not self.stop_event.is_set():
            try:
                frames = self.video_pipe.wait_for_frames(1000)
                depth = frames.get_depth_frame()
                left = frames.get_infrared_frame(1)
                right = frames.get_infrared_frame(2)
                if not depth or not left or not right:
                    continue
                seen += 1
                self.stats['video_frames'] += 1
                if seen <= warmup:
                    continue
                depth_np = np.asanyarray(depth.get_data()).copy()
                emitter_on = self._emitter.is_on(depth, depth_np)
                if emitter_on is None:
                    self.stats['unknown_emitter_frames'] += 1
                    continue
                depth_ts = _device_timestamp_ns(depth)
                if emitter_on:
                    self.queues.depth.put(DepthFrame(depth_ts, depth_np))
                    self.stats['depth_frames'] += 1
                else:
                    left_np = np.asanyarray(left.get_data()).copy()
                    right_np = np.asanyarray(right.get_data()).copy()
                    # cuVSLAM tracks the stereo IR images, so use the left IR
                    # hardware timestamp as the VIO timestamp rather than the
                    # depth frame timestamp from the same frameset.
                    vio_ts = _device_timestamp_ns(left)
                    self.queues.vio.put(VioFrame(vio_ts, left_np, right_np))
                    self.stats['vio_frames'] += 1
            except RuntimeError:
                if not self.stop_event.is_set():
                    time.sleep(0.01)
            except Exception:
                if not self.stop_event.is_set():
                    time.sleep(0.01)

    def _color_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                frames = self.color_pipe.wait_for_frames(1000)
                color = frames.get_color_frame()
                if not color:
                    continue
                arr = np.asanyarray(color.get_data()).copy()
                self.queues.color.put(ColorFrame(_device_timestamp_ns(color), arr))
                self.stats['color_frames'] += 1
            except RuntimeError:
                if not self.stop_event.is_set():
                    time.sleep(0.01)
            except Exception:
                if not self.stop_event.is_set():
                    time.sleep(0.01)

    def stop(self) -> None:
        self.stop_event.set()
        for pipe in (self.video_pipe, self.color_pipe, self.motion_pipe):
            try:
                pipe.stop()
            except Exception:
                pass
        for t in self.threads:
            if t.is_alive():
                t.join(timeout=1.0)
