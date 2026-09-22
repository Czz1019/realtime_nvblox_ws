from __future__ import annotations

import json
import time

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException, StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import MarkerArray

from realtime_nvblox.config import load_config
from realtime_nvblox.ros_publishers import esdf_cloud, mesh_markers, pose_stamped, transform_stamped
from realtime_nvblox.runtime import RealtimeNvbloxRuntime


class RealtimeNvbloxNode(Node):
    def __init__(self):
        super().__init__('realtime_nvblox')
        self.declare_parameter('config', '')
        config_path = str(self.get_parameter('config').value).strip() or None
        self.cfg = load_config(config_path)
        self.global_frame = self.cfg['frames']['global']
        self.rig_frame = self.cfg['frames']['rig']
        self.depth_frame = self.cfg['frames']['depth']
        self.color_frame = self.cfg['frames']['color']

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/realtime_nvblox/camera_pose', 10)
        self.path_pub = self.create_publisher(PathMsg, '/realtime_nvblox/camera_path', 2)
        self.mesh_pub = self.create_publisher(MarkerArray, '/realtime_nvblox/mesh', 1)
        self.slice_pub = self.create_publisher(PointCloud2, '/realtime_nvblox/esdf_slice', sensor_qos)
        self.esdf3d_pub = self.create_publisher(PointCloud2, '/realtime_nvblox/esdf_3d', sensor_qos)
        self.stats_pub = self.create_publisher(String, '/realtime_nvblox/stats', 2)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)

        self.path = PathMsg()
        self.path.header.frame_id = self.global_frame
        self.path_max = 3000

        self.runtime = RealtimeNvbloxRuntime(self.cfg)
        self.runtime.on('pose', self._on_pose)
        self.runtime.on('mesh', self._on_mesh)
        self.runtime.on('esdf_slice', self._on_slice)
        self.runtime.on('stats', self._on_stats)
        self.robot_mode = self.runtime.robot_pose is not None
        if self.robot_mode:
            if bool(self.get_parameter('use_sim_time').value):
                raise ValueError('Live RealSense robot mode requires use_sim_time: false and synchronized host clocks.')
            pcfg = self.cfg['robot_pose']
            if pcfg.get('input', 'tf') == 'tf':
                self.pose_tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
                self.pose_tf_listener = TransformListener(self.pose_tf_buffer, self)
                self.runtime.robot_pose.lookup = self._lookup_tool_pose
            else:
                msg_type = TransformStamped if pcfg.get('message_type', 'TransformStamped') == 'TransformStamped' else PoseStamped
                self.robot_pose_sub = self.create_subscription(msg_type, pcfg['topic'], self._on_robot_pose, sensor_qos)
        self.runtime.start()
        self._publish_static_extrinsics()

        self._last_esdf_sequence = 0
        self._last_esdf_version = 0
        publish_hz = float(self.cfg['esdf_3d'].get(
            'publish_hz', self.cfg['mapping'].get('esdf_3d_output_hz', 30.0)
        ))
        if publish_hz <= 0.0:
            publish_hz = 30.0
        self.esdf_timer = self.create_timer(1.0 / publish_hz, self._publish_latest_esdf3d)

        self.create_service(Trigger, '/realtime_nvblox/save_map', self._save_map)
        self.create_service(Trigger, '/realtime_nvblox/save_mesh', self._save_mesh)
        self.health_timer = self.create_timer(0.2, self._health)
        self.get_logger().info(
            'Python-first realtime nvblox started: mapping and full 3D ESDF query are decoupled.'
        )

    @staticmethod
    def _pose_matrix(translation, rotation):
        q = np.asarray([rotation.x, rotation.y, rotation.z, rotation.w], dtype=np.float64)
        if not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1.0, atol=1e-3):
            raise ValueError('Robot pose quaternion must be finite and normalized.')
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = Rotation.from_quat(q).as_matrix()
        T[:3, 3] = [translation.x, translation.y, translation.z]
        if not np.isfinite(T).all():
            raise ValueError('Robot pose translation must be finite.')
        return T

    def _lookup_tool_pose(self, timestamp_ns):
        cfg = self.cfg['robot_pose']
        try:
            transform = self.pose_tf_buffer.lookup_transform(
                cfg['base_frame'], cfg['tool_frame'], Time(nanoseconds=timestamp_ns),
                timeout=Duration(seconds=float(cfg.get('pose_wait_ms', 10.0)) / 1000.0),
            ).transform
            return self._pose_matrix(transform.translation, transform.rotation)
        except TransformException:
            self.runtime.stats.inc('robot_tf_unavailable')
            return None

    def _on_robot_pose(self, msg):
        cfg = self.cfg['robot_pose']
        try:
            if msg.header.frame_id != cfg['base_frame']:
                raise ValueError(f"Robot pose parent must be {cfg['base_frame']}.")
            if isinstance(msg, TransformStamped):
                if msg.child_frame_id != cfg['tool_frame']:
                    raise ValueError(f"Robot pose child must be {cfg['tool_frame']}.")
                T = self._pose_matrix(msg.transform.translation, msg.transform.rotation)
            else:
                # PoseStamped has no child frame: the configured topic must describe tool_frame.
                T = self._pose_matrix(msg.pose.position, msg.pose.orientation)
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if abs(time.time_ns() - stamp) > 5_000_000_000:
                raise ValueError('Robot pose must use acquisition time on the synchronized host clock.')
            self.runtime.add_robot_pose(stamp, T)
        except ValueError as exc:
            self.runtime.stats.inc('robot_pose_rejected')
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)

    def _publish_static_extrinsics(self):
        if self.robot_mode and not self.cfg['robot_pose'].get('publish_camera_extrinsics_tf', False):
            return
        c = self.runtime.calibration
        now_ns = int(self.get_clock().now().nanoseconds)
        self.static_tf_pub.sendTransform([
            transform_stamped(c.T_rig_depth, self.rig_frame, self.depth_frame, now_ns),
            transform_stamped(c.T_rig_color, self.rig_frame, self.color_frame, now_ns),
        ])

    def _on_pose(self, sample):
        msg = pose_stamped(sample.T_world_rig, self.global_frame, sample.timestamp_ns)
        self.pose_pub.publish(msg)
        self.path.header = msg.header
        self.path.poses.append(msg)
        if len(self.path.poses) > self.path_max:
            self.path.poses = self.path.poses[-self.path_max:]
        self.path_pub.publish(self.path)
        if not self.robot_mode or self.cfg['robot_pose'].get('publish_camera_pose_tf', False):
            self.tf_pub.sendTransform(
                transform_stamped(sample.T_world_rig, self.global_frame, self.rig_frame, sample.timestamp_ns)
            )

    def _on_mesh(self, mesh):
        self.mesh_pub.publish(mesh_markers(
            mesh.vertices, mesh.triangles, mesh.colors,
            self.global_frame, mesh.timestamp_ns,
            max_triangles=30000, chunk_triangles=10000,
        ))

    def _on_slice(self, data):
        max_d = float(self.cfg['esdf_slice'].get('max_visualized_distance_m', 2.0))
        self.slice_pub.publish(esdf_cloud(
            data.points_xyz, data.distance_m, self.global_frame, data.timestamp_ns, max_d
        ))

    def _publish_latest_esdf3d(self):
        sequence, data = self.runtime.esdf3d_results.get_if_newer(self._last_esdf_sequence)
        if data is None:
            return
        if sequence > self._last_esdf_sequence + 1:
            self.runtime.stats.inc('esdf_3d_publish_skipped', sequence - self._last_esdf_sequence - 1)
        self._last_esdf_sequence = sequence
        self._last_esdf_version = data.version

        max_d = float(self.cfg['esdf_3d'].get('max_visualized_distance_m', 2.0))
        start = time.perf_counter()
        full_grid = bool(self.cfg['esdf_3d'].get('publish_full_grid', True))
        unknown_d = None
        if self.runtime.nvblox is not None:
            unknown_d = float(getattr(self.runtime.nvblox, 'esdf_unknown_distance', 1.0e6))
        msg = esdf_cloud(
            data.points_xyz, data.distance_m, self.global_frame, data.timestamp_ns,
            max_d, full_grid=full_grid, unknown_distance_m=unknown_d
        )
        self.runtime.stats.add_latency('esdf_3d_ros_pack', (time.perf_counter() - start) * 1000.0)
        start = time.perf_counter()
        self.esdf3d_pub.publish(msg)
        self.runtime.stats.add_latency('esdf_3d_ros_publish', (time.perf_counter() - start) * 1000.0)
        self.runtime.stats.inc('esdf_3d_published')
        if self.robot_mode:
            self.runtime.stats.add_latency('esdf_3d_age', (time.time_ns() - data.timestamp_ns) / 1e6)

    def _on_stats(self, stats):
        msg = String()
        msg.data = json.dumps(stats, separators=(',', ':'), allow_nan=False)
        self.stats_pub.publish(msg)

    def _save_map(self, _req, res):
        try:
            path = self.runtime.save_map()
            res.success, res.message = True, str(path)
        except Exception as exc:
            res.success, res.message = False, repr(exc)
        return res

    def _save_mesh(self, _req, res):
        try:
            path = self.runtime.save_mesh()
            res.success, res.message = True, str(path)
        except Exception as exc:
            res.success, res.message = False, repr(exc)
        return res

    def _health(self):
        if self.runtime.error is not None:
            self.get_logger().fatal(f'Runtime failed: {self.runtime.error!r}')
            if rclpy.ok():
                rclpy.shutdown()

    def stop(self):
        self.runtime.stop()


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeNvbloxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node.runtime.error is not None:
        raise RuntimeError(node.runtime.error)


if __name__ == '__main__':
    main()
