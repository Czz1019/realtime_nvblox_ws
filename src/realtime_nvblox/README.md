# realtime_nvblox

Python-first real-time 3D reconstruction for NVIDIA GPUs and Intel RealSense D435i.

The runtime path is intentionally independent of Isaac ROS:

```text
D435i (pyrealsense2)
  ├─ emitter OFF IR1/IR2 ──> PyCuVSLAM stereo VIO
  ├─ emitter ON Depth ─────> nvblox_torch TSDF/ESDF/Mesh
  ├─ Color ────────────────> nvblox_torch color integration
  └─ IMU ─────────────────> PyCuVSLAM
```

ROS2 is optional. The core pipeline can run directly from Python. When ROS2 is available, `ros_mapper` publishes pose, path, mesh, 2D ESDF slice and local 3D ESDF point cloud to RViz2.

## What is retained from the Isaac ROS nvblox design

- RealSense projector on/off alternation.
- Clean stereo infrared frames for visual tracking.
- Active-depth frames for reconstruction.
- VIO pose + depth timestamp matching.
- Independent update rates for depth, color, mesh and ESDF.
- Bounded queues and low-latency processing.
- Continuous odometry frame for TSDF integration.

## What is removed

- Isaac ROS runtime.
- NITROS.
- `realsense_splitter` ROS component.
- `isaac_ros_visual_slam` ROS node.
- `isaac_ros_nvblox` ROS node.
- People segmentation/detection and other application-specific perception modules.

The equivalent splitter operation is implemented directly with librealsense `emitter_on_off` and per-frame emitter metadata.

## Outputs

ROS mode publishes:

- `/realtime_nvblox/camera_pose`
- `/realtime_nvblox/camera_path`
- `/realtime_nvblox/mesh`
- `/realtime_nvblox/esdf_slice`
- `/realtime_nvblox/esdf_3d`
- `/realtime_nvblox/stats`

Services:

- `/realtime_nvblox/save_map`
- `/realtime_nvblox/save_mesh`

Files:

- `realtime_map.nvblox`
- `realtime_mesh.ply`
- `runtime_stats.json`

## Recommended evaluation targets

On a laptop dGPU, first target:

- Stereo VIO: 25–30 Hz.
- Depth integration: 20–30 Hz.
- ESDF update: 5–10 Hz.
- Mesh internal update: 2–5 Hz.
- RViz mesh publishing: about 1 Hz.
- 3D ESDF local volume: about 1 Hz by default.

These are engineering targets, not guaranteed benchmark claims.

See `DEPLOY.md` for complete installation and validation.
