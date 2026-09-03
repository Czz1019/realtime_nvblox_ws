from __future__ import annotations

from importlib import metadata
import os
import platform
import sys

from realtime_nvblox.runtime_loader import (
    cuvslam_package_dir,
    nvblox_torch_package_dir,
    preload_cuvslam_runtime,
    preload_nvblox_runtime,
)


def _print_path_warnings() -> None:
    ld = os.environ.get('LD_LIBRARY_PATH', '')
    suspicious = [
        p for p in ld.split(':')
        if p and any(x in p for x in ('isaac_ros-dev', '/install/nvblox/', 'ros_pack'))
    ]
    if suspicious:
        print('WARNING: shell contains ROS/Isaac native-library paths:')
        for p in suspicious:
            print('  ', p)
        print('The runtime will explicitly preload wheel libraries, but a clean shell is still recommended.')


def main() -> None:
    print('=== realtime_nvblox environment check ===', flush=True)
    print('Python:', sys.executable, platform.python_version(), flush=True)
    print('setuptools:', metadata.version('setuptools'), flush=True)
    _print_path_warnings()

    import numpy as np
    print('NumPy:', np.__version__, flush=True)

    import torch
    print('Torch:', torch.__version__, flush=True)
    print('Torch CUDA:', torch.version.cuda, flush=True)
    print('CUDA available:', torch.cuda.is_available(), flush=True)
    if torch.cuda.is_available():
        print('GPU:', torch.cuda.get_device_name(0), flush=True)

    import pyrealsense2 as rs
    print('pyrealsense2:', getattr(rs, '__version__', metadata.version('pyrealsense2')), flush=True)
    ctx = rs.context()
    devices = list(ctx.query_devices())
    print('RealSense devices:', len(devices), flush=True)
    for d in devices:
        print(' -', d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number), flush=True)

    print('cuVSLAM wheel package:', cuvslam_package_dir(), flush=True)
    cuvslam_lib = preload_cuvslam_runtime()
    print('Preloaded cuVSLAM library:', cuvslam_lib, flush=True)
    import cuvslam
    print('cuVSLAM module:', cuvslam.__file__, flush=True)
    required = ['Tracker', 'Rig', 'Camera', 'ImuMeasurement']
    missing = [x for x in required if not hasattr(cuvslam, x)]
    if missing:
        raise RuntimeError(f'cuVSLAM API missing: {missing}')
    if not hasattr(cuvslam.Tracker, 'OdometryConfig'):
        raise RuntimeError('This package targets the cuVSLAM v15 Tracker.OdometryConfig API.')
    print('cuVSLAM v15-style API: OK', flush=True)

    print('nvblox_torch wheel package:', nvblox_torch_package_dir(), flush=True)
    nvblox_lib = preload_nvblox_runtime()
    print('Preloaded nvblox library:', nvblox_lib, flush=True)
    from nvblox_torch.mapper import Mapper
    from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType
    Mapper(voxel_sizes_m=0.05, integrator_types=ProjectiveIntegratorType.TSDF)
    print('nvblox_torch Mapper creation: OK', flush=True)
    print('Isaac ROS required: NO', flush=True)
    print('=== PASS ===', flush=True)


if __name__ == '__main__':
    main()
