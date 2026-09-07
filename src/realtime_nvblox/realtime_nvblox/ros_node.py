from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
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

    def _publish_static_extrinsics(self):
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
