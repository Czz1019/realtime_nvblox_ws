from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from realtime_nvblox.pose_buffer import PoseBuffer
from realtime_nvblox.types import PoseSample


def rigid_transform(value, name: str) -> np.ndarray:
    T = np.asarray(value, dtype=np.float64)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1.0, atol=1e-4)):
        raise ValueError(f'{name} must be a finite rigid 4x4 transform (translation in metres).')
    return T.astype(np.float32)


class RobotPoseProvider:
    """Interpolate base-from-tool before applying the hand-eye lever arm."""

    def __init__(self, cfg: dict, frames: dict):
        self.cfg = cfg
        self.input = cfg.get('input', 'tf')
        if self.input not in ('tf', 'topic'):
            raise ValueError('robot_pose.input must be tf or topic.')
        self.lookup = None
        required = ('base_frame', 'tool_frame', 'extrinsic_frame')
        if self.input == 'topic':
            required += ('topic',)
        for key in required:
            if not isinstance(cfg.get(key), str) or not cfg[key].strip():
                raise ValueError(f'robot_pose.{key} is required.')
        if cfg.get('message_type', 'TransformStamped') not in ('TransformStamped', 'PoseStamped'):
            raise ValueError('robot_pose.message_type must be TransformStamped or PoseStamped.')
        if cfg['extrinsic_frame'] not in (frames['rig'], frames['depth'], frames['color']):
            raise ValueError('robot_pose.extrinsic_frame must be the configured rig, depth or color optical frame.')
        handeye = cfg.get('handeye_transform')
        if handeye is not None and cfg.get('T_tool_camera') is not None:
            raise ValueError('Specify handeye_transform or T_tool_camera, not both.')
        if handeye is not None:
            translation = np.asarray(handeye.get('translation'), dtype=np.float64)
            quaternion = np.asarray(handeye.get('rotation'), dtype=np.float64)
            if (translation.shape != (3,) or not np.isfinite(translation).all()
                    or quaternion.shape != (4,) or not np.isfinite(quaternion).all()
                    or not np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-3)):
                raise ValueError('handeye_transform requires translation [x,y,z] metres and unit rotation [x,y,z,w].')
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
            T[:3, 3] = translation
        else:
            T = cfg.get('T_tool_camera')
        self.T_tool_camera = rigid_transform(T, 'robot_pose.T_tool_camera')
        world_base = cfg.get('T_world_base')
        if world_base is None and frames['global'] != cfg['base_frame']:
            raise ValueError('T_world_base is required when frames.global differs from robot_pose.base_frame.')
        self.T_world_base = rigid_transform(
            np.eye(4) if world_base is None else world_base, 'robot_pose.T_world_base'
        )
        if frames['global'] == cfg['base_frame'] and not np.allclose(self.T_world_base, np.eye(4)):
            raise ValueError('T_world_base must be identity when world and base are the same frame.')
        self.frames = frames
        self.buffer = PoseBuffer(int(cfg.get('buffer_size', 1024)))
        self.T_tool_rig = None

    def configure_calibration(self, calibration) -> None:
        frame = self.cfg['extrinsic_frame']
        T_rig_camera = np.eye(4, dtype=np.float32)
        if frame == self.frames['depth']:
            T_rig_camera = calibration.T_rig_depth
        elif frame == self.frames['color']:
            T_rig_camera = calibration.T_rig_color
        self.T_tool_rig = self.T_tool_camera @ np.linalg.inv(T_rig_camera)

    def add(self, timestamp_ns: int, T_base_tool: np.ndarray) -> bool:
        if timestamp_ns <= 0:
            raise ValueError('Robot pose must have a nonzero acquisition timestamp.')
        T = rigid_transform(T_base_tool, 'T_base_tool')
        return self.buffer.add(PoseSample(timestamp_ns, T))

    def query(self, timestamp_ns: int) -> PoseSample | None:
        if self.T_tool_rig is None:
            return None
        if self.input == 'tf':
            if self.lookup is None:
                raise RuntimeError('TF pose input requires the ROS node.')
            T = self.lookup(timestamp_ns)
            sample = None if T is None else PoseSample(timestamp_ns, rigid_transform(T, 'TF base-from-tool'))
        else:
            sample = self.buffer.query(
                timestamp_ns,
                max_error_ms=float(self.cfg.get('max_pose_error_ms', 20.0)),
                wait_ms=float(self.cfg.get('pose_wait_ms', 10.0)),
                allow_nearest=bool(self.cfg.get('allow_nearest', False)),
            )
        if sample is None or not sample.tracking_ok:
            return None
        return PoseSample(
            timestamp_ns, self.T_world_base @ sample.T_world_rig @ self.T_tool_rig,
            pose_error_ms=sample.pose_error_ms, interpolated=sample.interpolated,
        )
