from __future__ import annotations

import math
import struct

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation


def ns_to_stamp(timestamp_ns: int) -> TimeMsg:
    msg = TimeMsg()
    msg.sec = int(timestamp_ns // 1_000_000_000)
    msg.nanosec = int(timestamp_ns % 1_000_000_000)
    return msg


def header(frame_id: str, timestamp_ns: int) -> Header:
    h = Header()
    h.frame_id = frame_id
    h.stamp = ns_to_stamp(timestamp_ns)
    return h


def pose_stamped(T: np.ndarray, frame_id: str, timestamp_ns: int) -> PoseStamped:
    msg = PoseStamped()
    msg.header = header(frame_id, timestamp_ns)
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, q)
    return msg


def transform_stamped(T_parent_child: np.ndarray, parent: str, child: str, timestamp_ns: int) -> TransformStamped:
    msg = TransformStamped()
    msg.header = header(parent, timestamp_ns)
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, T_parent_child[:3, 3])
    q = Rotation.from_matrix(T_parent_child[:3, :3]).as_quat()
    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, q)
    return msg


def esdf_cloud(points: np.ndarray, distances: np.ndarray, frame_id: str, timestamp_ns: int, max_distance_m: float) -> PointCloud2:
    points = np.asarray(points, dtype=np.float32)
    d = np.asarray(distances, dtype=np.float32).reshape(-1)
    valid = np.isfinite(d) & (np.abs(d) <= float(max_distance_m))
    p = points[valid]
    d = d[valid]
    msg = PointCloud2()
    msg.header = header(frame_id, timestamp_ns)
    msg.height = 1
    msg.width = int(len(p))
    msg.is_bigendian = False
    msg.is_dense = False
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    if msg.width:
        packed = np.empty((msg.width, 4), dtype=np.float32)
        packed[:, :3] = p
        packed[:, 3] = d
        msg.data = packed.tobytes()
    else:
        msg.data = b''
    return msg


def mesh_markers(vertices: np.ndarray, triangles: np.ndarray, colors: np.ndarray, frame_id: str, timestamp_ns: int,
                 max_triangles: int = 30000, chunk_triangles: int = 10000) -> MarkerArray:
    vertices = np.asarray(vertices)
    triangles = np.asarray(triangles)
    colors = np.asarray(colors)
    if max_triangles > 0 and len(triangles) > max_triangles:
        step = max(1, int(math.ceil(len(triangles) / max_triangles)))
        triangles = triangles[::step][:max_triangles]
    out = MarkerArray()
    if len(triangles) == 0:
        return out
    for start in range(0, len(triangles), max(1, int(chunk_triangles))):
        tri_chunk = triangles[start:start + chunk_triangles]
        marker = Marker()
        marker.header = header(frame_id, timestamp_ns)
        marker.ns = 'realtime_nvblox_mesh'
        marker.id = start // max(1, int(chunk_triangles))
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        for tri in tri_chunk:
            for idx in tri:
                v = vertices[int(idx)]
                marker.points.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
                if colors.size:
                    c = colors[int(idx)]
                    from std_msgs.msg import ColorRGBA
                    if np.max(c) > 1.0:
                        c = c / 255.0
                    marker.colors.append(ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0))
        out.markers.append(marker)
    return out
