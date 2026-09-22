import os
from types import SimpleNamespace

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('RUN_GPU_TESTS') != '1', reason='Set RUN_GPU_TESTS=1 on a CUDA host.')


def test_depth_fusion_grid_query_and_buffer_ownership():
    from realtime_nvblox.nvblox_backend import NvbloxBackend
    from realtime_nvblox.types import CameraIntrinsics
    intr = CameraIntrinsics(64, 48, 60, 60, 31.5, 23.5)
    calibration = SimpleNamespace(depth_intrinsics=intr, color_intrinsics=intr, depth_scale_m=0.001)
    backend = NvbloxBackend(calibration, {'voxel_size_m': 0.05, 'max_integration_distance_m': 4})
    depth = np.full((48, 64), 1000, dtype=np.uint16)
    for _ in range(3):
        backend.integrate_depth(depth, np.eye(4))
    backend.update_esdf()
    cfg = dict(resolution_m=0.1, size_x_m=0.4, size_y_m=0.4, size_z_m=0.4)
    points, distances = backend.query_esdf_grid(np.array([0, 0, 0.8]), cfg)
    direct = backend.query_esdf(points)
    np.testing.assert_allclose(distances, direct, atol=1e-6)
    known = np.isfinite(distances) & (np.abs(distances) < 10)
    assert np.count_nonzero(known) > len(distances) / 2
    np.testing.assert_allclose(distances[known], 1.0 - points[known, 2], atol=0.11)
    saved = distances.copy()
    backend.query_esdf_grid(np.array([10, 10, 10]), cfg)
    np.testing.assert_array_equal(distances, saved)
