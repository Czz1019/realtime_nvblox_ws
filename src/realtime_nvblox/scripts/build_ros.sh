#!/usr/bin/env bash
set -euo pipefail

# This project needs ROS 2 only for the optional publisher/RViz layer.
# Never source isaac_ros-dev here.
source /opt/ros/humble/setup.bash
WS=${1:-$HOME/realtime_nvblox_ws}
cd "$WS"
rm -rf build/realtime_nvblox install/realtime_nvblox

# For deployment we intentionally use a normal install.  This avoids depending
# on setuptools' legacy "develop --editable" behavior.  If you specifically
# want symlink-install while developing, install setuptools<80 and run colcon
# manually with --symlink-install.
colcon build --packages-select realtime_nvblox
