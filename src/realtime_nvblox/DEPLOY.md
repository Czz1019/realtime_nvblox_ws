# Direct replacement deployment

This folder is intended to replace exactly:

`~/realtime_nvblox_ws/src/realtime_nvblox`

Do not keep an old copy with `package.xml` anywhere inside the colcon workspace;
colcon recursively discovers duplicate packages.

```bash
mkdir -p ~/realtime_nvblox_backup
mv ~/realtime_nvblox_ws/src/realtime_nvblox \
   ~/realtime_nvblox_backup/realtime_nvblox_$(date +%Y%m%d_%H%M%S)
cp -r ~/Downloads/realtime_nvblox ~/realtime_nvblox_ws/src/realtime_nvblox

cd ~/realtime_nvblox_ws
rm -rf build/realtime_nvblox install/realtime_nvblox
source /opt/ros/humble/setup.bash
colcon build --packages-select realtime_nvblox --event-handlers console_direct+
source ~/realtime_nvblox_ws/source_realtime_nvblox.sh

ros2 launch realtime_nvblox reconstruction.launch.py \
  config:=$HOME/realtime_nvblox_ws/src/realtime_nvblox/config/esdf_benchmark.yaml \
  run_rviz:=false
```

Second terminal:

```bash
source ~/realtime_nvblox_ws/source_realtime_nvblox.sh
ros2 topic hz /realtime_nvblox/esdf_3d
ros2 topic echo /realtime_nvblox/stats --once
```
