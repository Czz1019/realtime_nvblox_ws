#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/realtime_nvblox_ws/install/local_setup.bash" ]]; then
  source "$HOME/realtime_nvblox_ws/install/local_setup.bash"
fi

echo '--- nodes ---'
ros2 node list | grep realtime_nvblox || true

echo '--- topics ---'
ros2 topic list | grep realtime_nvblox || true

echo '--- pose rate ---'
timeout 6 ros2 topic hz /realtime_nvblox/camera_pose || true

echo '--- mesh rate ---'
timeout 6 ros2 topic hz /realtime_nvblox/mesh || true

echo '--- ESDF slice rate ---'
timeout 6 ros2 topic hz /realtime_nvblox/esdf_slice || true

echo '--- 3D ESDF rate ---'
timeout 6 ros2 topic hz /realtime_nvblox/esdf_3d || true

echo '--- stats ---'
timeout 3 ros2 topic echo /realtime_nvblox/stats --once || true
