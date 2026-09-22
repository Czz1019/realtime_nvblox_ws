import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from realtime_nvblox.config import load_config
from realtime_nvblox.nvblox_backend import NvbloxBackend
from realtime_nvblox.pose_buffer import PoseBuffer
from realtime_nvblox.rate_limiter import RateLimiter
from realtime_nvblox.robot_pose import RobotPoseProvider
from realtime_nvblox.types import DepthFrame, Esdf3DRequest, PoseSample


@pytest.mark.parametrize('input_hz', [30.0, 30.03, 30.1, 60.0])
def test_rate_does_not_halve_near_target(input_hz):
    limiter = RateLimiter(30.0)
    duration = 100.0
    count = sum(limiter.ready(100 + i / input_hz) for i in range(int(duration * input_hz)))
    assert abs(count / duration - 30.0) < 0.03


def test_rate_skips_missed_deadlines_without_burst():
    limiter = RateLimiter(30)
    assert limiter.ready(0)
    assert limiter.ready(10)
    assert not limiter.ready(10)
    assert limiter.remaining(10) == pytest.approx(1 / 30)


def test_exact_pose_and_strict_no_extrapolation():
    buffer = PoseBuffer()
    assert buffer.add(PoseSample(10_000_000, np.eye(4)))
    assert buffer.query(10_000_000, wait_ms=0, allow_nearest=False) is not None
    assert buffer.query(11_000_000, wait_ms=0, allow_nearest=False) is None
    assert buffer.query(11_000_000, wait_ms=0).pose_error_ms == 1
    assert not buffer.add(PoseSample(9_000_000, np.eye(4)))


def robot_config():
    return load_config(Path(__file__).parents[1] / 'robot_eye_in_hand.yaml')


def calibration():
    return SimpleNamespace(T_rig_depth=np.eye(4), T_rig_color=np.eye(4))


def test_provided_handeye_and_color_to_rig_conversion():
    cfg = robot_config()
    provider = RobotPoseProvider(cfg['robot_pose'], cfg['frames'])
    c = calibration()
    c.T_rig_color[:3, :3] = Rotation.from_euler('z', 20, degrees=True).as_matrix()
    c.T_rig_color[0, 3] = 0.02
    provider.configure_calibration(c)
    seen = []
    def lookup(stamp):
        seen.append(stamp)
        return np.eye(4)
    provider.lookup = lookup
    pose = provider.query(123)
    assert seen == [123]
    np.testing.assert_allclose(pose.T_world_rig @ c.T_rig_color, provider.T_tool_camera, atol=1e-6)
    np.testing.assert_allclose(provider.T_tool_camera[:3, 3], [-0.017628, -0.059898, -0.011736])
    provider.lookup = lambda ts: None
    assert provider.query(123) is None


def test_tool_interpolation_preserves_camera_lever_arm():
    cfg = robot_config()
    pcfg = cfg['robot_pose']
    pcfg.update(input='topic', topic='/pose', pose_wait_ms=0, max_pose_error_ms=600)
    pcfg.pop('handeye_transform')
    extrinsic = np.eye(4)
    extrinsic[0, 3] = 1
    pcfg['T_tool_camera'] = extrinsic
    provider = RobotPoseProvider(pcfg, cfg['frames'])
    provider.configure_calibration(calibration())
    a, b = np.eye(4), np.eye(4)
    b[:3, :3] = Rotation.from_euler('z', 180, degrees=True).as_matrix()
    provider.add(1_000_000_000, a)
    provider.add(2_000_000_000, b)
    pose = provider.query(1_500_000_000)
    np.testing.assert_allclose(pose.T_world_rig[:3, 3], [0, 1, 0], atol=1e-6)
    assert pose.interpolated
    assert provider.query(3_000_000_000) is None


def test_missing_or_invalid_handeye_rejected():
    cfg = robot_config()
    cfg['robot_pose'].pop('handeye_transform')
    with pytest.raises(ValueError):
        RobotPoseProvider(cfg['robot_pose'], cfg['frames'])
    cfg['robot_pose']['T_tool_camera'] = np.zeros((4, 4))
    with pytest.raises(ValueError):
        RobotPoseProvider(cfg['robot_pose'], cfg['frames'])


@pytest.mark.parametrize('resolution', [0.15, 0.10])
def test_grid_symmetric_inside_box_with_exact_spacing(resolution):
    cfg = dict(resolution_m=resolution, size_x_m=4, size_y_m=4, size_z_m=2)
    grid = NvbloxBackend.grid_offsets(cfg)
    for dim, size in enumerate([4, 4, 2]):
        axis = np.unique(grid[:, dim])
        assert axis[0] >= -size / 2 - 1e-6
        assert axis[-1] <= size / 2 + 1e-6
        np.testing.assert_allclose(axis, -axis[::-1], atol=1e-6)
        np.testing.assert_allclose(np.diff(axis), resolution, atol=1e-6)


def test_grid_invalid_resolution_rejected():
    with pytest.raises(ValueError):
        NvbloxBackend.grid_offsets(dict(resolution_m=0, size_x_m=4, size_y_m=4, size_z_m=2))


def test_realsense_rotation_matches_sdk():
    import pyrealsense2 as rs
    from realtime_nvblox.realsense_source import _extrinsics_matrix
    ext = rs.extrinsics()
    ext.rotation = [0, 1, 0, -1, 0, 0, 0, 0, 1]
    ext.translation = [0.1, 0.2, 0.3]
    source = SimpleNamespace(get_extrinsics_to=lambda target: ext)
    T = _extrinsics_matrix(source, None)
    actual = (T @ [1, 0, 0, 1])[:3]
    np.testing.assert_allclose(actual, rs.rs2_transform_point_to_point(ext, [1, 0, 0]))


def runtime(monkeypatch):
    import realtime_nvblox.runtime as module
    monkeypatch.setattr(module, 'RealSenseSource', lambda *a, **kw: SimpleNamespace(stats={}, error=None))
    rt = module.RealtimeNvbloxRuntime(load_config())
    rt.calibration = calibration()
    rt.nvblox = SimpleNamespace(lock=threading.RLock(), query_timings_ms={})
    return rt


def test_query_keeps_early_request_and_uses_latest_snapshot(monkeypatch):
    rt = runtime(monkeypatch)
    rt.cfg['esdf_3d']['generate_hz'] = 5
    queried = []
    def query(center, cfg):
        queried.append(center.copy())
        return np.zeros((1, 3)), np.zeros(1)
    rt.nvblox.query_esdf_grid = query
    def request(version):
        with rt.nvblox.lock:
            rt._latest_esdf_request = Esdf3DRequest(version * 10, np.full(3, version), version)
            rt.esdf3d_requests.put(rt._latest_esdf_request)
    request(1)
    worker = threading.Thread(target=rt._esdf3d_loop_guarded)
    worker.start()
    try:
        seq, first = rt.esdf3d_results.wait_newer(0, timeout=2)
        assert first.version == 1
        request(2)
        request(3)
        _, result = rt.esdf3d_results.wait_newer(seq, timeout=2)
        assert result is not None  # No fourth request needed to wake pending work.
        assert result.version == 3 and result.timestamp_ns == 30
        np.testing.assert_array_equal(queried[-1], [3, 3, 3])
        assert rt.error is None
    finally:
        rt.stop_event.set()
        rt.esdf3d_requests.close()
        worker.join(2)
    assert not worker.is_alive()


def test_dirty_esdf_updated_without_another_depth_frame(monkeypatch):
    rt = runtime(monkeypatch)
    rt.cfg['mapping'].update(depth_hz=0, esdf_update_hz=5, enable_mesh=False, enable_color=False)
    rt._pose_for = lambda ts: PoseSample(ts, np.eye(4))
    rt._publish_stats = lambda: None
    rt.nvblox.integrate_depth = lambda *args: None
    first_update = threading.Event()
    updates = []
    def update():
        updates.append(time.monotonic())
        first_update.set()
        if len(updates) == 2:
            rt.stop_event.set()
    rt.nvblox.update_esdf = update
    rt.queues.depth.put(DepthFrame(1, np.ones((2, 2))))
    worker = threading.Thread(target=rt._mapping_loop_guarded)
    worker.start()
    try:
        assert first_update.wait(2)
        rt.queues.depth.put(DepthFrame(2, np.ones((2, 2))))
        worker.join(2)
        assert len(updates) == 2
        assert rt._latest_esdf_request.timestamp_ns == 2
        assert rt.error is None
    finally:
        rt.stop_event.set()
        worker.join(2)


def test_robot_timestamp_domain_checked():
    import pyrealsense2 as rs
    from realtime_nvblox.realsense_source import RealSenseSource
    source = RealSenseSource.__new__(RealSenseSource)
    source.robot_mode = True
    source.stats = {'timestamp_drops': 0}
    source.error = None
    source.stop_event = threading.Event()
    frame = SimpleNamespace(get_frame_timestamp_domain=lambda: rs.timestamp_domain.hardware_clock,
                            get_timestamp=lambda: 1234)
    assert source._timestamp_ns(frame) is None
    assert source.stats['timestamp_drops'] == 1
    stamp = time.time_ns()
    frame.get_frame_timestamp_domain = lambda: rs.timestamp_domain.global_time
    frame.get_timestamp = lambda: stamp / 1e6
    assert abs(source._timestamp_ns(frame) - stamp) < 1024


def test_robot_video_accepts_every_depth_without_ir_or_emitter_classifier():
    from realtime_nvblox.realsense_source import RealSenseSource
    from realtime_nvblox.queues import DropOldestQueue
    source = RealSenseSource.__new__(RealSenseSource)
    source.cfg = {'warmup_frames': 0, 'emitter_flashing': False}
    source.robot_mode = True
    source.stats = dict(video_frames=0, depth_frames=0)
    source.stop_event = threading.Event()
    source.queues = SimpleNamespace(depth=DropOldestQueue(8))
    source._timestamp_ns = lambda frame: 123
    depth = SimpleNamespace(get_data=lambda: np.ones((2, 2), dtype=np.uint16))
    frames = SimpleNamespace(get_depth_frame=lambda: depth,
                             get_infrared_frame=lambda index: None)
    calls = []
    def wait(timeout):
        calls.append(1)
        if len(calls) == 4:
            source.stop_event.set()
        return frames
    source.video_pipe = SimpleNamespace(wait_for_frames=wait)
    source._video_loop()
    assert source.stats['depth_frames'] == 4
    assert source.queues.depth.qsize() == 4


def test_robot_runtime_start_does_not_initialize_cuvslam(monkeypatch):
    import realtime_nvblox.runtime as module
    source = SimpleNamespace(start=calibration, stop=lambda: None)
    monkeypatch.setattr(module, 'RealSenseSource', lambda *a, **kw: source)
    def forbidden(*args):
        pytest.fail('Robot mode must not initialize cuVSLAM')
    monkeypatch.setattr(module, 'CuVslamBackend', forbidden)
    monkeypatch.setattr(module, 'NvbloxBackend', lambda *a: SimpleNamespace(prepare_esdf_grid=lambda cfg: None))
    names = []
    monkeypatch.setattr(module.threading, 'Thread', lambda **kw: SimpleNamespace(start=lambda: names.append(kw['name'])))
    rt = module.RealtimeNvbloxRuntime(robot_config())
    rt.start()
    assert rt.vslam is None
    assert names == ['nvblox-mapping', 'nvblox-esdf3d']


def test_rate_alternating_arrival_jitter_does_not_halve():
    limiter = RateLimiter(30)
    count = sum(limiter.ready(100 + i / 30 + (0.002 if i % 2 == 0 else -0.002))
                for i in range(3000))
    assert count >= 2998
