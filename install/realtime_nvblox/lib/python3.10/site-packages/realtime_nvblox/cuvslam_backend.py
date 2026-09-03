from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from realtime_nvblox.runtime_loader import preload_cuvslam_runtime
from realtime_nvblox.types import Calibration, ImuSample, PoseSample, VioFrame


class CuVslamBackend:
    """PyCuVSLAM v15-compatible stereo VIO wrapper.

    The rig frame is the left infrared optical frame. Odometry is used for
    nvblox fusion because it is continuous. Optional asynchronous SLAM is kept
    for global-pose diagnostics without forcing loop-closure jumps into TSDF.
    """

    def __init__(self, calibration: Calibration, cfg: dict):
        self.loaded_lib = preload_cuvslam_runtime()
        import cuvslam

        self.vslam = cuvslam
        self.calib = calibration
        self.cfg = cfg
        self.tracker = self._create_tracker()
        self.registered_imu = 0
        self.tracked_frames = 0
        self.failed_frames = 0

    @staticmethod
    def _pose_from_matrix(vslam, T: np.ndarray):
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        return vslam.Pose(rotation=q, translation=T[:3, 3])

    def _camera(self, intr, T_rig_camera: np.ndarray | None = None):
        cam = self.vslam.Camera()
        cam.distortion = self.vslam.Distortion(self.vslam.Distortion.Model.Pinhole)
        cam.focal = (float(intr.fx), float(intr.fy))
        cam.principal = (float(intr.cx), float(intr.cy))
        cam.size = (int(intr.width), int(intr.height))
        if T_rig_camera is not None:
            cam.rig_from_camera = self._pose_from_matrix(self.vslam, T_rig_camera)
        return cam

    def _imu(self):
        imu = self.vslam.ImuCalibration()
        T = self.calib.T_rig_imu.copy()
        # Match NVIDIA's RealSense VIO example by default: use calibrated
        # translation while keeping the IMU orientation identity. Full rotation
        # can be enabled only when its convention has been independently verified.
        if not bool(self.cfg.get('use_full_imu_rotation', False)):
            T[:3, :3] = np.eye(3, dtype=np.float32)
        imu.rig_from_imu = self._pose_from_matrix(self.vslam, T)
        imu.gyroscope_noise_density = float(self.cfg.get('gyro_noise_density', 0.0060673370376614875))
        imu.gyroscope_random_walk = float(self.cfg.get('gyro_random_walk', 3.6211951458325785e-05))
        imu.accelerometer_noise_density = float(self.cfg.get('accel_noise_density', 0.0336219792080528))
        imu.accelerometer_random_walk = float(self.cfg.get('accel_random_walk', 9.825658997185147e-04))
        imu.frequency = float(self.calib.imu_frequency_hz)
        return imu

    def _create_tracker(self):
        rig = self.vslam.Rig()
        rig.cameras = [
            self._camera(self.calib.left_intrinsics),
            self._camera(self.calib.right_intrinsics, self.calib.T_rig_right),
        ]
        rig.imus = [self._imu()]

        cfg = self.vslam.Tracker.OdometryConfig(
            async_sba=bool(self.cfg.get('async_sba', False)),
            enable_final_landmarks_export=False,
            enable_observations_export=False,
            debug_imu_mode=False,
            odometry_mode=self.vslam.Tracker.OdometryMode.Inertial,
            rectified_stereo_camera=bool(self.cfg.get('rectified_stereo', True)),
        )
        if bool(self.cfg.get('enable_slam', True)):
            slam_cfg = self.vslam.Tracker.SlamConfig(
                sync_mode=bool(self.cfg.get('slam_sync_mode', False))
            )
            return self.vslam.Tracker(rig, cfg, slam_cfg)
        return self.vslam.Tracker(rig, cfg)

    def register_imu(self, sample: ImuSample) -> None:
        m = self.vslam.ImuMeasurement()
        m.timestamp_ns = int(sample.timestamp_ns)
        m.linear_accelerations = np.asarray(sample.acceleration_mps2, dtype=np.float64)
        m.angular_velocities = np.asarray(sample.angular_velocity_rps, dtype=np.float64)
        self.tracker.register_imu_measurement(0, m)
        self.registered_imu += 1

    @staticmethod
    def _matrix_from_pose(pose) -> np.ndarray:
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = Rotation.from_quat(np.asarray(pose.rotation, dtype=np.float64)).as_matrix().astype(np.float32)
        T[:3, 3] = np.asarray(pose.translation, dtype=np.float32)
        return T

    def track(self, frame: VioFrame) -> PoseSample | None:
        odom_est, slam_est = self.tracker.track(
            int(frame.timestamp_ns),
            (np.asarray(frame.left, dtype=np.uint8), np.asarray(frame.right, dtype=np.uint8)),
        )
        if odom_est.world_from_rig is None:
            self.failed_frames += 1
            return None
        odom_T = self._matrix_from_pose(odom_est.world_from_rig.pose)
        slam_T = None
        if slam_est is not None:
            try:
                if slam_est.world_from_rig is not None:
                    slam_T = self._matrix_from_pose(slam_est.world_from_rig.pose)
            except AttributeError:
                try:
                    slam_T = self._matrix_from_pose(slam_est.pose)
                except Exception:
                    slam_T = None
        self.tracked_frames += 1
        return PoseSample(int(frame.timestamp_ns), odom_T, True, slam_T)
