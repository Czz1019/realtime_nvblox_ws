from __future__ import annotations

import math

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
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


def _distance_to_rgb_u32(
    distances: np.ndarray,
    max_distance_m: float,
    unknown_mask: np.ndarray | None = None,
) -> np.ndarray:
    """
    Convert absolute ESDF distance to packed 0x00RRGGBB colors.

    Color scale:
        0.00 * max_distance -> red
        0.25 * max_distance -> yellow
        0.50 * max_distance -> green
        0.75 * max_distance -> cyan
        1.00 * max_distance -> blue
        unknown              -> dark gray

    Negative ESDF values (inside an obstacle) are deliberately mapped to red.
    """
    d = np.asarray(distances, dtype=np.float32).reshape(-1)
    max_d = max(float(max_distance_m), 1.0e-6)
    t = np.clip(np.abs(d) / max_d, 0.0, 1.0)

    rgb = np.zeros((d.shape[0], 3), dtype=np.uint8)

    m0 = t < 0.25
    if np.any(m0):
        u = t[m0] / 0.25
        rgb[m0, 0] = 255
        rgb[m0, 1] = np.clip(255.0 * u, 0.0, 255.0).astype(np.uint8)

    m1 = (t >= 0.25) & (t < 0.50)
    if np.any(m1):
        u = (t[m1] - 0.25) / 0.25
        rgb[m1, 0] = np.clip(255.0 * (1.0 - u), 0.0, 255.0).astype(np.uint8)
        rgb[m1, 1] = 255

    m2 = (t >= 0.50) & (t < 0.75)
    if np.any(m2):
        u = (t[m2] - 0.50) / 0.25
        rgb[m2, 1] = 255
        rgb[m2, 2] = np.clip(255.0 * u, 0.0, 255.0).astype(np.uint8)

    m3 = t >= 0.75
    if np.any(m3):
        u = np.clip((t[m3] - 0.75) / 0.25, 0.0, 1.0)
        rgb[m3, 1] = np.clip(255.0 * (1.0 - u), 0.0, 255.0).astype(np.uint8)
        rgb[m3, 2] = 255

    # Any signed distance <= 0 is inside/on an obstacle: force red.
    inside = np.isfinite(d) & (d <= 0.0)
    rgb[inside, 0] = 255
    rgb[inside, 1] = 0
    rgb[inside, 2] = 0

    if unknown_mask is not None:
        unknown_mask = np.asarray(unknown_mask, dtype=bool).reshape(-1)
        rgb[unknown_mask] = np.array([55, 55, 55], dtype=np.uint8)

    return (
        (rgb[:, 0].astype(np.uint32) << 16)
        | (rgb[:, 1].astype(np.uint32) << 8)
        | rgb[:, 2].astype(np.uint32)
    )


def esdf_cloud(
    points: np.ndarray,
    distances: np.ndarray,
    frame_id: str,
    timestamp_ns: int,
    max_distance_m: float,
    full_grid: bool = False,
    unknown_distance_m: float | None = None,
) -> PointCloud2:
    """
    Pack ESDF into a single colored PointCloud2.

    Fields are:
        x, y, z, rgb, distance

    The rgb field is derived directly from |distance| so RViz can use the
    RGB8 color transformer while the original signed ESDF distance remains
    available to downstream algorithms.

    full_grid=True preserves the complete query grid. Unknown nvblox samples
    are kept and drawn dark gray instead of being mistaken for very-safe blue.
    """
    points = np.asarray(points, dtype=np.float32)
    d = np.asarray(distances, dtype=np.float32).reshape(-1)

    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] != d.shape[0]:
        raise ValueError(f'points/distance shape mismatch: {points.shape}, {d.shape}')

    finite = np.isfinite(d)
    if unknown_distance_m is None or not np.isfinite(float(unknown_distance_m)):
        # Fallback for nvblox_torch if the constant was not exposed.
        # Current backend fallback is 1e6, so this is deliberately generous.
        unknown = (~finite) | (np.abs(d) >= 1.0e5)
    else:
        unknown_value = abs(float(unknown_distance_m))
        tolerance = max(1.0e-3, unknown_value * 1.0e-5)
        unknown = (~finite) | (np.abs(np.abs(d) - unknown_value) <= tolerance)

    if full_grid:
        valid = np.ones(d.shape, dtype=bool)
    else:
        # Visualization/lightweight mode: omit unknown cells and keep only
        # the requested ESDF distance band.
        valid = finite & (~unknown) & (np.abs(d) <= float(max_distance_m))

    p = points[valid]
    dv = d[valid]
    unknown_v = unknown[valid]

    packed_rgb_u32 = _distance_to_rgb_u32(
        dv,
        max_distance_m=max_distance_m,
        unknown_mask=unknown_v,
    )
    # PCL/RViz convention: rgb is FLOAT32 carrying packed RGB bits.
    packed_rgb_f32 = packed_rgb_u32.view(np.float32)

    msg = PointCloud2()
    msg.header = header(frame_id, timestamp_ns)
    msg.height = 1
    msg.width = int(p.shape[0])
    msg.is_bigendian = False
    msg.is_dense = False
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='distance', offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 20
    msg.row_step = msg.point_step * msg.width

    if msg.width:
        packed = np.empty((msg.width, 5), dtype=np.float32)
        packed[:, :3] = p
        packed[:, 3] = packed_rgb_f32
        packed[:, 4] = dv
        msg.data = packed.tobytes(order='C')
    else:
        msg.data = b''

    return msg


def mesh_markers(vertices: np.ndarray, triangles: np.ndarray, colors: np.ndarray,
                 frame_id: str, timestamp_ns: int,
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
    from std_msgs.msg import ColorRGBA
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
                    if np.max(c) > 1.0:
                        c = c / 255.0
                    marker.colors.append(ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0))
        out.markers.append(marker)
    return out
