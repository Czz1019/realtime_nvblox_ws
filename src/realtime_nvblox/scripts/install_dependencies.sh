#!/usr/bin/env bash
set -euo pipefail

PYTHON=${PYTHON:-python3}
PIP=("$PYTHON" -m pip install --no-cache-dir)
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  PIP+=(--user)
fi

echo '[1/7] Python packaging tools'
# IMPORTANT: ROS 2 Humble/colcon --symlink-install is currently incompatible
# with setuptools >= 80 because colcon still invokes setup.py develop --editable.
# Do NOT upgrade setuptools to latest here.
"${PIP[@]}" --upgrade pip wheel 'setuptools<80'

echo '[2/7] Numeric + RealSense dependencies'
"${PIP[@]}" numpy==1.26.4 'scipy>=1.10,<1.16' 'PyYAML>=6.0' \
  pyrealsense2==2.57.7.10387

echo '[3/7] PyTorch CUDA 12.8'
"${PIP[@]}" --index-url https://download.pytorch.org/whl/cu128 \
  torch==2.9.1 torchvision==0.24.1

echo '[4/7] nvblox_torch official Ubuntu 22.04 CUDA12 wheel'
"${PIP[@]}" --no-deps \
  'https://github.com/nvidia-isaac/nvblox/releases/download/v0.0.10/nvblox_torch-0.0.10%2Bcu12ubuntu22-py3-none-linux_x86_64.whl'

echo '[5/7] PyCuVSLAM v15 official Ubuntu 22.04/Python3.10/CUDA12 wheel'
"${PIP[@]}" --no-deps \
  'https://github.com/nvidia-isaac/cuVSLAM/releases/download/v15.0.0/cuvslam-15.0.0%2Bcu12-cp310-cp310-manylinux_2_35_x86_64.whl'

echo '[6/7] Remaining nvblox_torch Python dependencies'
"${PIP[@]}" 'open3d>=0.19' 'opencv-python>=4.8' 'einops>=0.7' 'imageio>=2.30' \
  'matplotlib>=3.7' 'nvtx>=0.2' 'timm>=1.0' 'transforms3d>=0.4'

echo '[7/7] Install this package editable for standalone use'
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
"$PYTHON" -m pip install --user -e "$ROOT"

echo
echo 'Dependency installation finished.'
echo 'IMPORTANT: for standalone testing, open a NEW terminal and do NOT source isaac_ros-dev.'
echo 'Then run:'
echo '  cd ~/realtime_nvblox_ws/src/realtime_nvblox'
echo '  python3 -m realtime_nvblox.check_env'
