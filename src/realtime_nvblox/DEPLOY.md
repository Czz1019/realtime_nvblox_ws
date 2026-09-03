# Complete deployment guide

> **重要环境规则**：Standalone / Python-first 测试必须在新终端运行，**不要 source `~/workspaces/isaac_ros-dev/install/setup.bash`**。本工程不依赖 Isaac ROS；source 它反而可能让 `libcuvslam.so` / `libnvblox_lib.so` 被错误版本抢先加载。

> ROS2 只在 RViz 发布阶段 source `/opt/ros/humble/setup.bash`。部署构建默认使用普通 `colcon build`，不要求 `--symlink-install`。如果开发时需要 symlink-install，请保持 `setuptools<80`。

This guide deliberately does **not** source or install Isaac ROS.

The intended machine is Ubuntu 22.04, Python 3.10, CUDA 12, NVIDIA GPU and Intel RealSense D435i. ROS2 Humble is only needed for the optional RViz2 adapter.

---

## 1. Recommended clean workspace

Using a clean workspace proves that the project has no hidden dependency on an old ROS nvblox build.

```bash
mkdir -p ~/realtime_nvblox_ws/src
cp -a ~/Downloads/realtime_nvblox ~/realtime_nvblox_ws/src/
cd ~/realtime_nvblox_ws/src/realtime_nvblox
```

Do **not** source:

```bash
source ~/workspaces/isaac_ros-dev/install/setup.bash
```

The package does not need it.

---

## 2. Install Python GPU dependencies

```bash
cd ~/realtime_nvblox_ws/src/realtime_nvblox
./scripts/install_dependencies.sh
```

The script installs the integration baseline:

- NumPy 1.26.4
- PyTorch 2.9.1 + CUDA 12.8 wheel
- torchvision 0.24.1
- nvblox_torch 0.0.10 Ubuntu 22 / CUDA 12 wheel
- PyCuVSLAM 15.0.0 Ubuntu 22 / Python 3.10 / CUDA 12 wheel
- pyrealsense2 2.57.7

The exact CUDA driver/toolkit must still be compatible with the wheels.

---

## 3. Install the package as a normal Python package

This step is enough for standalone operation; ROS is not involved.

```bash
cd ~/realtime_nvblox_ws/src/realtime_nvblox
python3 -m pip install --user -e .
```

Check imports:

```bash
python3 -m realtime_nvblox.check_env
```

Expected key lines:

```text
CUDA available: True
cuVSLAM v15-style API: OK
Preloaded nvblox library: ...site-packages/nvblox_torch/lib/nvblox/libnvblox_lib.so
nvblox_torch Mapper creation: OK
Isaac ROS required: NO
=== PASS ===
```

The explicit library preload is important. The process loads the `libnvblox_lib.so` bundled with the `nvblox_torch` wheel before `libpy_nvblox.so`. Therefore a stale ROS nvblox installation cannot silently replace the wheel library.

---

## 4. Test standalone reconstruction first

Connect the D435i directly over USB 3.

Run:

```bash
cd ~/realtime_nvblox_ws/src/realtime_nvblox
./scripts/run_standalone.sh
```

or:

```bash
standalone_mapper --config ~/realtime_nvblox_ws/src/realtime_nvblox/config/default.yaml
```

No ROS node is required in this mode.

The terminal prints periodic JSON statistics. Important counters are:

```text
rates_hz.vio_pose
rates_hz.depth_integrated
rates_hz.color_integrated
rates_hz.esdf_updated
rates_hz.mesh_updated
queue_drops
latency.vio_track
latency.depth_integrate
latency.esdf_update
latency.mesh_update
```

Stop with Ctrl+C. By default the map and mesh are written to:

```text
~/nvblox_output/realtime_map.nvblox
~/nvblox_output/realtime_mesh.ply
~/nvblox_output/runtime_stats.json
```

This is the most important migration test: if this works, the complete mapping chain is independent of ROS and Isaac ROS.

---

## 5. How the Python splitter works

The default configuration requests:

```text
Depth/IR: 640x480 @ 60 Hz
Color:    640x480 @ 15 Hz
```

For the stereo depth sensor, the program enables:

```text
emitter_enabled = 1
emitter_on_off  = 1
```

Librealsense then alternates projector state between frames. The program checks per-frame emitter metadata:

```text
Emitter OFF -> IR1 + IR2 -> cuVSLAM
Emitter ON  -> Depth     -> nvblox
```

This is the same functional idea as the Isaac ROS RealSense splitter, but implemented without ROS messages or a splitter component.

If emitter metadata is unavailable, the code can estimate the ON/OFF frame parity from the valid-depth ratio. For maximum repeatability, set:

```yaml
realsense:
  emitter_metadata_required: true
```

once your D435i firmware is confirmed to expose `frame_emitter_mode`.

---

## 6. VIO and depth synchronization

cuVSLAM receives only emitter-off stereo frames. Depth frames occur on alternating emitter-on frames, so their timestamps generally fall between VIO poses.

The runtime stores recent VIO poses and waits briefly for the next pose. It then interpolates translation and rotation to the exact depth timestamp before fusion:

```text
IR pose(t0) ---- Depth(td) ---- IR pose(t1)
                    |
                    +--> interpolate T_world_rig(td)
```

Then sensor extrinsics are applied:

```text
T_world_depth = T_world_rig @ T_rig_depth
T_world_color = T_world_rig @ T_rig_color
```

This avoids pretending that Depth, IR and Color share exactly the same optical center.

---

## 7. Build the optional ROS2/RViz2 adapter

Only now source normal ROS2 Humble:

```bash
source /opt/ros/humble/setup.bash
cd ~/realtime_nvblox_ws
colcon build --packages-select realtime_nvblox
source ~/realtime_nvblox_ws/install/local_setup.bash
```

Again, do not source Isaac ROS.

Check:

```bash
ros2 pkg prefix realtime_nvblox
ros2 pkg executables realtime_nvblox
```

Expected executables include:

```text
realtime_nvblox standalone_mapper
realtime_nvblox ros_mapper
realtime_nvblox check_environment
```

---

## 8. One-command ROS/RViz run

```bash
source /opt/ros/humble/setup.bash
source ~/realtime_nvblox_ws/install/local_setup.bash

ros2 launch realtime_nvblox reconstruction.launch.py
```

The launch file starts only:

```text
ros_mapper
rviz2
```

`ros_mapper` itself directly opens the RealSense device and directly calls PyCuVSLAM and nvblox_torch.

There is no:

```text
realsense2_camera ROS node
realsense_splitter ROS node
isaac_ros_visual_slam
isaac_ros_nvblox
NITROS
```

---

## 9. Verify ROS outputs

```bash
ros2 topic list | grep realtime_nvblox
```

Expected:

```text
/realtime_nvblox/camera_pose
/realtime_nvblox/camera_path
/realtime_nvblox/mesh
/realtime_nvblox/esdf_slice
/realtime_nvblox/esdf_3d
/realtime_nvblox/stats
```

Check pose/VIO rate:

```bash
ros2 topic hz /realtime_nvblox/camera_pose
```

Check mesh:

```bash
ros2 topic hz /realtime_nvblox/mesh
```

Check 2D ESDF slice:

```bash
ros2 topic hz /realtime_nvblox/esdf_slice
```

Check local 3D ESDF:

```bash
ros2 topic hz /realtime_nvblox/esdf_3d
```

Stats:

```bash
ros2 topic echo /realtime_nvblox/stats --once
```

Or run:

```bash
~/realtime_nvblox_ws/src/realtime_nvblox/scripts/check_runtime.sh
```

---

## 10. RViz2

The bundled RViz configuration uses:

```text
Fixed Frame = odom
```

and shows:

- Mesh: `/realtime_nvblox/mesh`
- ESDF slice: `/realtime_nvblox/esdf_slice`
- Camera path: `/realtime_nvblox/camera_path`
- Camera pose: `/realtime_nvblox/camera_pose`

The 3D ESDF display is present but disabled by default because a dense local 3D point cloud can clutter visualization. Enable `ESDF 3D` when needed.

---

## 11. 3D ESDF design

`nvblox_torch` keeps the ESDF on the GPU. Exporting every ESDF voxel in an ever-growing global world is expensive and unnecessary for local robot planning.

Therefore `/realtime_nvblox/esdf_3d` is a configurable local query volume around the camera/robot. Default:

```yaml
esdf_3d:
  resolution_m: 0.15
  size_x_m: 4.0
  size_y_m: 4.0
  size_z_m: 2.0
```

For a manipulator, later change the center from the camera to the robot workspace center or end-effector planning region.

For collision checking inside an algorithm, call `NvbloxBackend.query_esdf()` or `query_esdf_with_gradients()` directly instead of converting the field to a ROS point cloud.

---

## 12. Save map and mesh in ROS mode

```bash
ros2 service call /realtime_nvblox/save_map std_srvs/srv/Trigger '{}'
ros2 service call /realtime_nvblox/save_mesh std_srvs/srv/Trigger '{}'
```

Files are written under:

```text
~/nvblox_output/
```

---

## 13. Recommended initial performance settings

Do not start with maximum output bandwidth. Use:

```yaml
mapping:
  voxel_size_m: 0.05
  max_integration_distance_m: 4.0
  depth_hz: 30.0
  color_hz: 10.0
  esdf_update_hz: 10.0
  mesh_update_hz: 5.0
  mesh_output_hz: 1.0
  esdf_slice_output_hz: 2.0
  esdf_3d_output_hz: 1.0
```

Once stable, increase only the rate needed by the downstream planner.

---

## 14. Accuracy tuning

The configuration exposes RealSense IMU noise terms. The default values follow the PyCuVSLAM RealSense example. If you have camera-specific Allan-variance calibration, replace:

```yaml
vslam:
  gyro_noise_density: ...
  gyro_random_walk: ...
  accel_noise_density: ...
  accel_random_walk: ...
```

Do this only after the basic pipeline is stable.

---

## 15. Why nvblox uses odometry instead of loop-closure jumps

The runtime can enable asynchronous cuVSLAM SLAM, but TSDF integration uses continuous odometry. Loop closure can correct historical poses discontinuously. A previously integrated TSDF cannot automatically move all old voxels to corrected positions.

If globally loop-closed reconstruction is required later, implement one of:

1. keyframe depth storage + map reintegration;
2. independent TSDF submaps attached to a pose graph;
3. periodic rebuild after loop closure.

Do not simply replace continuous odometry with a jumping global SLAM pose during live TSDF fusion.

---

## 16. Five-minute acceptance test

Run:

```bash
watch -n 1 nvidia-smi
```

and in another terminal:

```bash
ros2 topic echo /realtime_nvblox/stats
```

Acceptance:

```text
[ ] no crash
[ ] VIO remains active while moving
[ ] camera path is continuous
[ ] existing walls stay fixed in odom
[ ] depth queue does not grow indefinitely
[ ] queue drops are bounded rather than continuously exploding
[ ] GPU memory stabilizes
[ ] Mesh continues updating
[ ] ESDF continues updating
[ ] map and PLY save successfully
```

---

## 17. Thirty-minute stability test

After the five-minute run succeeds, repeat for 30 minutes and inspect `runtime_stats.json`.

The most important metrics are VIO failure count, pose drops, depth integration latency p95, ESDF latency p95, mesh-to-CPU latency and dropped queue counts.

---

## 18. Migration to another machine

The migration unit is the single directory:

```text
realtime_nvblox/
```

Target machine requirements:

```text
Ubuntu 22.04
Python 3.10
NVIDIA driver + CUDA 12
D435i + USB3
```

Then:

```bash
./scripts/install_dependencies.sh
python3 -m pip install --user -e .
python3 -m realtime_nvblox.check_env
```

For standalone operation that is enough.

For RViz2 additionally install ROS2 Humble and build this one package with colcon.

No Isaac ROS workspace is part of the migration procedure.
