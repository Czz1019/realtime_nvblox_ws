# realtime_nvblox 1.2.0

Single ROS2 Humble Python package for Intel RealSense D435i + PyCuVSLAM v15 +
`nvblox_torch` realtime mapping.  It directly opens the D435i with
`pyrealsense2`; it does **not** require Isaac ROS, NITROS, or
`realsense2_camera` at runtime.

Main data path:

```
D435i IR/IMU -> cuVSLAM -> pose buffer
D435i depth + interpolated pose -> TSDF -> ESDF update
                                         -> latest-only full 3D ESDF worker
                                         -> /realtime_nvblox/esdf_3d
```

Key ROS topics:
- `/realtime_nvblox/camera_pose`
- `/realtime_nvblox/camera_path`
- `/realtime_nvblox/esdf_3d` (`sensor_msgs/PointCloud2`, fields x/y/z/distance)
- `/realtime_nvblox/esdf_slice`
- `/realtime_nvblox/mesh`
- `/realtime_nvblox/stats`

Use `config/esdf_benchmark.yaml` first.  The default launch keeps RViz disabled
so mapping performance can be verified before visualization is added.
