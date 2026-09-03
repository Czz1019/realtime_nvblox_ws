from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class Calibration:
    left_intrinsics: CameraIntrinsics
    right_intrinsics: CameraIntrinsics
    depth_intrinsics: CameraIntrinsics
    color_intrinsics: CameraIntrinsics
    T_rig_right: np.ndarray
    T_rig_depth: np.ndarray
    T_rig_color: np.ndarray
    T_rig_imu: np.ndarray
    depth_scale_m: float
    imu_frequency_hz: float


@dataclass
class ImuSample:
    timestamp_ns: int
    acceleration_mps2: np.ndarray
    angular_velocity_rps: np.ndarray


@dataclass
class VioFrame:
    timestamp_ns: int
    left: np.ndarray
    right: np.ndarray


@dataclass
class DepthFrame:
    timestamp_ns: int
    depth_raw: np.ndarray


@dataclass
class ColorFrame:
    timestamp_ns: int
    color_rgb: np.ndarray


@dataclass
class PoseSample:
    timestamp_ns: int
    T_world_rig: np.ndarray
    tracking_ok: bool = True
    slam_T_world_rig: Optional[np.ndarray] = None


@dataclass
class MeshData:
    timestamp_ns: int
    vertices: np.ndarray
    triangles: np.ndarray
    colors: np.ndarray


@dataclass
class EsdfData:
    timestamp_ns: int
    points_xyz: np.ndarray
    distance_m: np.ndarray
