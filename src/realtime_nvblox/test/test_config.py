from realtime_nvblox.config import load_config


def test_default_config():
    cfg = load_config()
    assert cfg['mapping']['voxel_size_m'] > 0
    assert cfg['vslam']['enable'] is True
    assert cfg['realsense']['emitter_flashing'] is True
