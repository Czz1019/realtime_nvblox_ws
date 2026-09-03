import numpy as np
from realtime_nvblox.pose_buffer import PoseBuffer
from realtime_nvblox.types import PoseSample


def test_pose_interpolation():
    b = PoseBuffer(8)
    T0 = np.eye(4, dtype=np.float32)
    T1 = np.eye(4, dtype=np.float32)
    T1[0, 3] = 1.0
    b.add(PoseSample(0, T0))
    b.add(PoseSample(1_000_000_000, T1))
    p = b.query(500_000_000, max_error_ms=600, wait_ms=0)
    assert p is not None
    assert abs(float(p.T_world_rig[0, 3]) - 0.5) < 1e-5
