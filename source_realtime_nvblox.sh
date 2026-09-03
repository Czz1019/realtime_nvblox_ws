#!/usr/bin/env bash

# ============================================================
# realtime_nvblox ROS2 clean environment
# ============================================================

# 清除其它 ROS workspace / Isaac ROS overlay
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset PYTHONPATH

unset ROS_DISTRO
unset ROS_VERSION
unset ROS_PYTHON_VERSION

unset AMENT_CURRENT_PREFIX
unset COLCON_CURRENT_PREFIX
unset LD_PRELOAD

# 基础 PATH
export PATH="$HOME/.local/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# CUDA
export LD_LIBRARY_PATH="/usr/local/cuda/lib64"

# ------------------------------------------------------------
# ROS2 Humble
# ------------------------------------------------------------
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[ERROR] ROS2 Humble not found"
    return 1 2>/dev/null || exit 1
fi

source /opt/ros/humble/setup.bash

# ------------------------------------------------------------
# cuVSLAM wheel
# ------------------------------------------------------------
CUV_DIR=$(python3 - <<'PY'
from importlib import metadata
from pathlib import Path

dist = metadata.distribution("cuvslam")
print(Path(dist.locate_file("cuvslam")).resolve())
PY
)

# ------------------------------------------------------------
# nvblox_torch wheel
# ------------------------------------------------------------
NVB_DIR=$(python3 - <<'PY'
from importlib import metadata
from pathlib import Path

dist = metadata.distribution("nvblox-torch")
print(Path(dist.locate_file("nvblox_torch")).resolve())
PY
)

# ------------------------------------------------------------
# Torch native libraries
# ------------------------------------------------------------
TORCH_LIB=$(python3 - <<'PY'
import os
import torch

print(
    os.path.join(
        os.path.dirname(torch.__file__),
        "lib"
    )
)
PY
)

# wheel中的native library必须优先于其它ROS库
export LD_LIBRARY_PATH="$CUV_DIR:$NVB_DIR/lib/nvblox:$TORCH_LIB:$LD_LIBRARY_PATH"

# ------------------------------------------------------------
# realtime_nvblox workspace overlay
# ------------------------------------------------------------
RT_WS="$HOME/realtime_nvblox_ws"

if [ -f "$RT_WS/install/local_setup.bash" ]; then
    source "$RT_WS/install/local_setup.bash"
else
    echo "[WARNING] realtime_nvblox workspace has not been built."
fi

echo
echo "====================================================="
echo " realtime_nvblox ROS2 environment"
echo "====================================================="
echo "ROS_DISTRO : ${ROS_DISTRO:-NOT_SET}"
echo "Python     : $(which python3)"
echo "cuVSLAM    : $CUV_DIR"
echo "nvblox     : $NVB_DIR"
echo "Torch libs : $TORCH_LIB"
echo "Workspace  : $RT_WS"
echo "====================================================="
echo
