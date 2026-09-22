from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer

from realtime_nvblox.ros_node import RealtimeNvbloxNode
from realtime_nvblox.stats import RuntimeStats


def test_ros_tf_lookup_uses_exposure_time_and_rejects_extrapolation():
    buffer = Buffer()
    for sec, x in [(1, 0.0), (2, 1.0)]:
        msg = TransformStamped()
        msg.header.frame_id = 'base_link'
        msg.child_frame_id = 'tool0'
        msg.header.stamp.sec = sec
        msg.transform.rotation.w = 1.0
        msg.transform.translation.x = x
        buffer.set_transform(msg, 'test')
    adapter = SimpleNamespace(
        cfg={'robot_pose': {'base_frame': 'base_link', 'tool_frame': 'tool0', 'pose_wait_ms': 0}},
        pose_tf_buffer=buffer,
        runtime=SimpleNamespace(stats=RuntimeStats()),
        _pose_matrix=RealtimeNvbloxNode._pose_matrix,
    )
    T = RealtimeNvbloxNode._lookup_tool_pose(adapter, 1_500_000_000)
    np.testing.assert_allclose(T[:3, 3], [0.5, 0, 0])
    assert RealtimeNvbloxNode._lookup_tool_pose(adapter, 3_000_000_000) is None
    assert adapter.runtime.stats.counts['robot_tf_unavailable'] == 1
