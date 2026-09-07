# realtime_nvblox_ws

> 基于仓库 `Czz1019/realtime_nvblox_ws` 的当前 `main` 分支整理。  
> 分析基准：commit `1fe9255facb845dc20070383c677770aa27275b9`。  
> 本文档重点说明 **系统数据流、各 Python 模块职责、线程/队列关系、配置参数、ROS2 接口、运行方式以及当前实现的关键限制**。

---

## 1. 项目简介

`realtime_nvblox_ws` 是一个面向 **Intel RealSense D435i + NVIDIA GPU** 的 Python-first 实时三维重建工程。

核心技术栈：

- **pyrealsense2**：直接读取 D435i 深度、双目红外、彩色和 IMU 数据；
- **PyCuVSLAM / cuVSLAM 15**：利用双目红外 + IMU 估计连续相机位姿；
- **nvblox_torch**：在 GPU 上完成 TSDF 融合、Color 融合、ESDF 更新与 Mesh 更新；
- **ROS2 Humble / rclpy**：作为可选输出层，把 Pose、Path、Mesh、ESDF 和统计信息发布到 RViz2；
- **不依赖 Isaac ROS Runtime / NITROS / isaac_ros_nvblox 节点**。

项目整体思想是：

```text
                     Intel RealSense D435i
                              │
          ┌───────────────────┼───────────────────┐
          │                   │                   │
      IR1 + IR2             Depth               Color
          │                   │                   │
          │                   │                   │
          └──────┐            │                   │
                 │            │                   │
                IMU           │                   │
                 │            │                   │
                 ▼            │                   │
              cuVSLAM         │                   │
                 │            │                   │
          T_world_rig(t)      │                   │
                 │            │                   │
                 └──────┬─────┴──────────────┬────┘
                        │                    │
                        ▼                    ▼
                时间戳匹配 / 位姿插值       Color Pose
                        │
                        ▼
                  nvblox_torch Mapper
                        │
                    TSDF Layer
                   /           \
                  ▼             ▼
              ESDF Update    Mesh Update
                  │             │
          ┌───────┴───────┐     ▼
          ▼               ▼    Mesh
      2D ESDF Query   3D ESDF Query
          │               │
          └───────┬───────┘
                  ▼
             ROS2 / RViz2
```

---

# 2. 本工程与 Isaac ROS nvblox 的关系

本工程不是直接启动：

```text
realsense2_camera
realsense_splitter
isaac_ros_visual_slam
isaac_ros_nvblox
NITROS
```

而是将类似功能直接放进 Python 进程：

```text
Isaac ROS组件思想                 本工程实现
──────────────────────────────────────────────────────────
RealSense驱动                    pyrealsense2
RealSense Splitter               EmitterClassifier + emitter_on_off
Visual SLAM                      PyCuVSLAM
Depth/Pose时间对齐               PoseBuffer
nvblox mapping                   nvblox_torch.Mapper
RViz消息转换                     ros_publishers.py
ROS节点                          ros_node.py
```

因此：

> **重建核心可以不启动 ROS2，ROS2 只负责输出、TF 和 RViz 可视化。**

---

# 3. 仓库总体结构

当前仓库结构可以概括为：

```text
realtime_nvblox_ws/
│
├── build/                         # colcon 构建中间文件
├── install/                       # colcon 安装结果
├── log/                           # colcon 构建日志
│
├── source_realtime_nvblox.sh      # 清理环境并加载 ROS/CUDA/native wheel 库
│
└── src/
    └── realtime_nvblox/
        │
        ├── README.md
        ├── DEPLOY.md
        ├── LICENSE
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── requirements.txt
        │
        ├── config/
        │   └── default.yaml
        │
        ├── launch/
        │   └── reconstruction.launch.py
        │
        ├── rviz/
        │   └── realtime_nvblox.rviz
        │
        ├── scripts/
        │   ├── install_dependencies.sh
        │   ├── build_ros.sh
        │   ├── run_standalone.sh
        │   ├── start_clean.sh
        │   ├── check_environment.sh
        │   ├── check_runtime.sh
        │   └── diagnose_native_libraries.sh
        │
        ├── test/
        │   ├── test_config.py
        │   └── test_pose_buffer.py
        │
        └── realtime_nvblox/
            ├── __init__.py
            ├── types.py
            ├── config.py
            ├── queues.py
            ├── rate_limiter.py
            ├── stats.py
            ├── runtime_loader.py
            ├── check_env.py
            ├── realsense_source.py
            ├── cuvslam_backend.py
            ├── pose_buffer.py
            ├── nvblox_backend.py
            ├── runtime.py
            ├── ros_publishers.py
            ├── ros_node.py
            ├── cli.py
            └── default.yaml
```

其中真正的业务源码集中在：

```text
src/realtime_nvblox/realtime_nvblox/
```

而仓库根目录的：

```text
build/
install/
log/
```

属于 `colcon` 自动生成目录，不属于算法源码。

---

# 4. 核心运行链路

系统启动后主要建立两条工作链：

```text
RealtimeNvbloxRuntime
│
├── VIO线程
│   └── _vio_loop()
│
└── Mapping线程
    └── _mapping_loop()
```

同时 `RealSenseSource` 内部还运行：

```text
RealSenseSource
│
├── rs-depth-ir thread
├── rs-color thread
└── RealSense motion callback
```

因此系统并不是单线程串行执行。

---

# 5. RealSense 数据采集部分

文件：

```text
realtime_nvblox/realsense_source.py
```

这是整个系统的传感器入口。

## 5.1 RealSenseSource

核心类：

```python
class RealSenseSource
```

负责：

1. 查找 D435i；
2. 建立 Depth / IR / Color / IMU stream；
3. 读取相机内参；
4. 读取各传感器之间外参；
5. 控制 projector / emitter；
6. 根据 emitter 状态分流数据；
7. 把结果写入不同的有界队列。

## 5.2 三条 RealSense pipeline

代码中不是所有数据都放在同一个 pipeline 中，而是：

```text
video_pipe
    ├─ Depth
    ├─ Infrared 1
    └─ Infrared 2

color_pipe
    └─ Color

motion_pipe
    ├─ Accelerometer
    └─ Gyroscope
```

这样做的目的主要是避免：

```text
15 Hz Color
```

限制：

```text
60 Hz Depth / Stereo IR
```

## 5.3 默认 RealSense 参数

`config/default.yaml` 中：

```yaml
realsense:
  depth_width: 640
  depth_height: 480
  depth_fps: 60

  color_width: 640
  color_height: 480
  color_fps: 15

  preferred_gyro_fps: 200
  preferred_accel_fps: 200
```

系统会尝试找到最接近设置值的实际 IMU profile。

---

# 6. Projector / Emitter 分流

类：

```python
EmitterClassifier
```

这是本工程中替代 Isaac ROS `realsense_splitter` 的关键部分。

D435i 设置：

```text
emitter_enabled = 1
emitter_on_off  = 1
```

之后深度模组交替运行 projector 状态。

系统根据 emitter 状态把帧分为：

```text
Emitter ON
    ↓
Depth
    ↓
nvblox 重建

Emitter OFF
    ↓
IR1 + IR2
    ↓
cuVSLAM
```

也就是：

```text
有投影器干扰的帧 → 用于主动深度
无投影器干扰的帧 → 用于双目视觉跟踪
```

## 6.1 Emitter metadata

优先读取：

```text
frame_emitter_mode
frame_laser_power_mode
```

如果 firmware 不提供 metadata，则根据：

```text
偶数帧有效深度比例
奇数帧有效深度比例
```

估计哪一类 parity 是 emitter ON。

相关配置：

```yaml
emitter_flashing: true
emitter_metadata_required: false
parity_calibration_frames: 20
```

---

# 7. 标定数据

数据结构定义在：

```text
types.py
```

核心：

```python
Calibration
```

保存：

```text
left_intrinsics
right_intrinsics
depth_intrinsics
color_intrinsics

T_rig_right
T_rig_depth
T_rig_color
T_rig_imu

depth_scale_m
imu_frequency_hz
```

工程选择：

```text
rig frame = left infrared optical frame
```

因此：

```text
T_rig_depth
```

表示 Depth Camera 到 Rig 的标定变换。

对应建图变换：

```python
T_world_depth = pose.T_world_rig @ calibration.T_rig_depth
T_world_color = pose.T_world_rig @ calibration.T_rig_color
```

这样 Depth、IR、Color 不会被错误地认为拥有完全相同的光心。

---

# 8. 数据类型定义

文件：

```text
types.py
```

定义了整个工程内部的数据接口。

| 类型 | 含义 |
|---|---|
| `CameraIntrinsics` | 相机宽高、fx、fy、cx、cy |
| `Calibration` | 全部相机内参与传感器外参 |
| `ImuSample` | 时间戳、加速度、角速度 |
| `VioFrame` | 左右红外双目图像 |
| `DepthFrame` | 原始深度图 |
| `ColorFrame` | RGB 图像 |
| `PoseSample` | `T_world_rig` 位姿 |
| `MeshData` | Mesh 顶点、三角形、颜色 |
| `EsdfData` | 查询点坐标与 ESDF 距离 |

这些 dataclass 是模块之间交换数据的标准格式。

---

# 9. 有界队列

文件：

```text
queues.py
```

核心类：

```python
DropOldestQueue
```

特点：

```text
队列未满
    ↓
正常加入

队列已满
    ↓
删除最旧数据
    ↓
加入最新数据
```

目标不是保证每一帧都处理，而是：

> **实时系统优先保留最新数据，避免不断积累延迟。**

系统使用的主要队列：

```text
vio
depth
color
imu
```

默认：

```yaml
vio_queue_size: 8
depth_queue_size: 4
color_queue_size: 2
imu_queue_size: 2000
```

每个队列都有 `dropped` 计数，可以在统计信息里查看丢帧情况。

---

# 10. cuVSLAM 后端

文件：

```text
cuvslam_backend.py
```

核心类：

```python
CuVslamBackend
```

主要职责：

```text
Calibration
     ↓
构造 Rig
     ↓
Stereo Camera
     +
IMU Calibration
     ↓
cuVSLAM Tracker
     ↓
Odometry Pose
```

## 10.1 双目模型

Rig 中加入：

```text
Left IR Camera
Right IR Camera
```

右相机使用标定外参配置 `rig_from_camera`。

## 10.2 IMU 标定

IMU 参数包括：

```yaml
gyro_noise_density
gyro_random_walk
accel_noise_density
accel_random_walk
```

代码默认：

```python
use_full_imu_rotation: false
```

因此默认使用已标定平移、但把 IMU rotation 设为 identity；完整旋转只在明确验证坐标定义后再开启。

## 10.3 cuVSLAM 模式

使用：

```python
Tracker.OdometryConfig
```

默认：

```text
OdometryMode.Inertial
```

可以额外打开：

```yaml
enable_slam: true
```

用于生成 SLAM 结果。

实时 TSDF 融合使用连续 odometry `T_world_rig`。

---

# 11. VIO 时间戳处理

VIO 的主要逻辑位于：

```text
runtime.py::_vio_loop()
```

cuVSLAM 要求 IMU 与 image 共享严格单调递增时间线。

代码维护：

```python
last_tracker_ts
```

## 11.1 IMU 注册规则

对于当前图像时间 `frame_ts`，只注册：

```text
imu_ts < frame_ts
```

不会先注册 `imu_ts == frame_ts` 再调用 `track(frame_ts)`。

## 11.2 过期数据

如果：

```text
timestamp <= last_tracker_ts
```

就丢弃，并统计：

```text
imu_timestamp_drop
vio_timestamp_drop
```

用于避免帧乱序或重复时间戳破坏 tracker 时间线。

---

# 12. PoseBuffer：Depth / Color 与 VIO 对齐

文件：

```text
pose_buffer.py
```

核心类：

```python
PoseBuffer
```

cuVSLAM 产生：

```text
IR pose(t0)
IR pose(t1)
```

Depth 可能位于：

```text
t0 < td < t1
```

系统进行：

```text
       Depth(td)
          │
IR(t0)────┼────IR(t1)
          │
          ▼
      Pose(td)
```

平移使用线性插值；旋转使用 `scipy.spatial.transform.Slerp`。

当前 `PoseBuffer.query()` 还会等待未来 pose 到达。

默认：

```yaml
pose_wait_ms: 35.0
max_pose_error_ms: 60.0
```

如果无法插值，再在允许误差范围内取最近位姿。

---

# 13. nvblox 后端

文件：

```text
nvblox_backend.py
```

核心类：

```python
NvbloxBackend
```

负责：

```text
Depth / Color / Pose
        ↓
nvblox_torch Mapper
        ↓
TSDF / Color / ESDF / Mesh
```

## 13.1 Mapper 初始化

使用：

```python
Mapper(
    voxel_sizes_m=...,
    integrator_types=ProjectiveIntegratorType.TSDF
)
```

默认：

```yaml
voxel_size_m: 0.05
```

即地图 voxel 边长为 5 cm。

注意：

> `voxel_size_m=0.05` 表示地图离散分辨率，并不等价于重建误差保证为 5 cm。

---

# 14. Depth 融合

调用：

```python
self.mapper.add_depth_frame(...)
```

之前完成：

```text
RealSense uint16 depth
       ↓
float32 CUDA tensor
       ↓
乘 depth_scale_m
       ↓
米
       ↓
无效深度清零
       ↓
超出 max_integration_distance_m 清零
```

默认：

```yaml
max_integration_distance_m: 4.0
```

Depth tensor 保持在 GPU；Pose 按当前 `nvblox_torch` API 转成 CPU float32 tensor。

---

# 15. Color 融合

调用：

```python
self.mapper.add_color_frame(...)
```

Color 转成 CUDA `uint8` tensor，并配合：

```text
T_world_color
```

完成颜色融合。

默认：

```yaml
enable_color: true
color_hz: 10.0
```

---

# 16. ESDF 更新

调用：

```python
self.mapper.update_esdf(0)
```

当前默认：

```yaml
enable_esdf: true
esdf_update_hz: 10.0
```

ESDF 是 nvblox GPU map 的内部距离场。

---

# 17. Mesh 更新

调用：

```python
self.mapper.update_color_mesh(0)
```

默认：

```yaml
enable_mesh: true
mesh_update_hz: 5.0
```

当需要发送 ROS 时：

```python
mesh_numpy()
```

把顶点、三角形和颜色转成 CPU NumPy。

所以：

```text
mesh_update_hz ≠ mesh_output_hz
```

默认：

```yaml
mesh_update_hz: 5.0
mesh_output_hz: 1.0
```

---

# 18. Mapper RLock

`NvbloxBackend` 内部使用：

```python
threading.RLock()
```

保护：

```text
integrate_depth
integrate_color
update_esdf
update_mesh
mesh_numpy
query_esdf
save_map
save_mesh
```

因此这些对同一个 Mapper 的操作不会并发访问。

优点是线程安全关系简单；代价是 ESDF 查询或 Mesh CPU 输出较慢时可能延长其它 Mapper 操作等待时间。

---

# 19. ESDF 查询接口

## 19.1 普通查询

```python
query_esdf(points_xyz)
```

输入通常是：

```text
N × 3
```

由于当前 `nvblox_torch v0.0.10` C++ binding 对 ESDF query 实际要求 `N × 4`，代码自动补：

```text
radius = 0
```

所以：

```text
[x, y, z]
```

变成：

```text
[x, y, z, 0]
```

## 19.2 带梯度查询

```python
query_esdf_with_gradients()
```

可返回 ESDF gradient + distance；当前 ROS 实时可视化没有直接使用该接口。

---

# 20. 当前 2D ESDF Slice 的实际实现

这是理解当前工程非常重要的一点。

文件：

```text
runtime.py
```

函数：

```python
_grid_2d()
_output_esdf_slice()
```

当前 Slice **不是直接读取 nvblox 原生 ESDF voxel slice**。

而是：

```text
指定中心
   ↓
按照 resolution_m 创建规则 XY 查询网格
   ↓
query_esdf()
   ↓
得到距离
   ↓
PointCloud2
```

默认：

```yaml
esdf_slice:
  resolution_m: 0.10
  size_x_m: 8.0
  size_y_m: 8.0
```

因此地图 `voxel_size` 与 Slice 显示点间距是两个独立参数。

---

# 21. 当前 3D ESDF 输出的实际实现

函数：

```python
_grid_3d()
_output_esdf_3d()
```

数据流：

```text
中心点
   ↓
生成 3D np.meshgrid
   ↓
N × 3 Query Points
   ↓
补 radius=0
   ↓
GPU query_esdf
   ↓
distance GPU→CPU
   ↓
ROS PointCloud2
```

默认：

```yaml
esdf_3d:
  resolution_m: 0.15
  size_x_m: 4.0
  size_y_m: 4.0
  size_z_m: 2.0
```

所以默认 3D ESDF 是相机附近固定尺寸的局部规则查询体，并不是把 nvblox 内部全部 ESDF voxel 原样发布。

---

# 22. `voxel_size` 与 ESDF 输出 `resolution` 的区别

当前工程必须区分：

```yaml
mapping:
  voxel_size_m: 0.05
```

和：

```yaml
esdf_slice:
  resolution_m: 0.10
```

以及：

```yaml
esdf_3d:
  resolution_m: 0.15
```

| 参数 | 作用 |
|---|---|
| `mapping.voxel_size_m` | nvblox 内部 TSDF / ESDF 地图分辨率 |
| `esdf_slice.resolution_m` | 自定义二维 ESDF 查询网格采样间距 |
| `esdf_3d.resolution_m` | 自定义三维 ESDF 查询网格采样间距 |

所以把：

```text
voxel_size_m: 0.05 → 0.03
```

而 `esdf_3d.resolution_m` 仍为 `0.15` 时，ROS 端 3D ESDF 点密度不会按地图 voxel_size 成比例增加。

这不是内部 ESDF 没变密，而是输出采样分辨率被独立定义了。

---

# 23. RateLimiter

文件：

```text
rate_limiter.py
```

逻辑：

```python
if now - last >= period:
    last = now
    return True
```

Runtime 为以下任务分别建立 limiter：

```text
depth_lim
color_lim
esdf_lim
mesh_lim
mesh_out_lim
slice_out_lim
esdf3d_out_lim
stats_lim
```

因此系统有多层频率：

```text
传感器输入率
    ↓
Depth 融合率
    ↓
ESDF 更新率
    ↓
ESDF 输出率
```

不能只根据：

```bash
ros2 topic hz /realtime_nvblox/esdf_3d
```

判断地图内部 ESDF 更新速度。

---

# 24. 默认 Mapping 频率

```yaml
mapping:
  depth_hz: 30.0
  color_hz: 10.0
  esdf_update_hz: 10.0
  mesh_update_hz: 5.0
  mesh_output_hz: 1.0
  esdf_slice_output_hz: 2.0
  esdf_3d_output_hz: 1.0
  stats_hz: 1.0
```

对应目标：

```text
Depth fusion       30 Hz
Color fusion       10 Hz
ESDF update        10 Hz
Mesh update         5 Hz
ESDF Slice output   2 Hz
3D ESDF output      1 Hz
Mesh ROS output     1 Hz
```

这些是调度上限/目标，实际频率仍取决于传感器、VIO、Pose 匹配、GPU 计算、CPU copy 和 ROS 发布耗时。

---

# 25. Runtime 主控制器

文件：

```text
runtime.py
```

类：

```python
RealtimeNvbloxRuntime
```

职责：

```text
创建队列
  ↓
启动 RealSense
  ↓
启动 cuVSLAM
  ↓
启动 nvblox
  ↓
启动 VIO Thread
  ↓
启动 Mapping Thread
  ↓
管理 callback
  ↓
统计性能
  ↓
保存地图 / Mesh
  ↓
处理退出
```

---

# 26. Runtime callbacks

支持：

```python
runtime.on('pose', ...)
runtime.on('mesh', ...)
runtime.on('esdf_slice', ...)
runtime.on('esdf_3d', ...)
runtime.on('stats', ...)
```

因此核心 Runtime 不直接依赖 ROS。

Standalone：

```text
Runtime → 打印 Stats
```

ROS：

```text
Runtime → ROS callback → ROS topic
```

---

# 27. ROS2 Node

文件：

```text
ros_node.py
```

类：

```python
RealtimeNvbloxNode
```

这个 Node **不订阅 RealSense ROS topic**。

而是：

```text
ros_mapper
   ↓
RealtimeNvbloxRuntime
   ↓
pyrealsense2 直接打开 D435i
```

所以启动本包时不需要额外启动 `realsense2_camera`。

---

# 28. ROS Topics

当前发布：

```text
/realtime_nvblox/camera_pose
/realtime_nvblox/camera_path
/realtime_nvblox/mesh
/realtime_nvblox/esdf_slice
/realtime_nvblox/esdf_3d
/realtime_nvblox/stats
```

## 28.1 camera_pose

```text
geometry_msgs/PoseStamped
```

表示当前 `T_global_rig`。

## 28.2 camera_path

```text
nav_msgs/Path
```

最多保留 3000 个 pose。

## 28.3 mesh

```text
visualization_msgs/MarkerArray
```

通过 `TRIANGLE_LIST` 发布；代码限制最多约 30000 triangles，并按 10000 triangles/Marker 分块。

## 28.4 esdf_slice

```text
sensor_msgs/PointCloud2
```

字段：

```text
x
y
z
intensity
```

其中：

```text
intensity = ESDF distance
```

## 28.5 esdf_3d

同样是 `PointCloud2`，坐标来自 `_grid_3d()`，距离来自 `query_esdf()`。

## 28.6 stats

```text
std_msgs/String
```

内容是 JSON。

---

# 29. ROS Services

当前：

```text
/realtime_nvblox/save_map
/realtime_nvblox/save_mesh
```

均为：

```text
std_srvs/srv/Trigger
```

调用示例：

```bash
ros2 service call \
  /realtime_nvblox/save_map \
  std_srvs/srv/Trigger '{}'
```

---

# 30. TF

ROS Node 发布动态：

```text
global_frame → rig_frame
```

并发布静态：

```text
rig_frame → depth_frame
rig_frame → color_frame
```

默认：

```yaml
frames:
  global: odom
  rig: camera_left_optical_frame
  depth: camera_depth_optical_frame
  color: camera_color_optical_frame
```

---

# 31. ros_publishers.py

专门负责：

```text
NumPy / Internal Data
          ↓
ROS Message
```

主要函数：

```python
ns_to_stamp()
header()
pose_stamped()
transform_stamped()
esdf_cloud()
mesh_markers()
```

这样 ROS 消息格式转换不会混进 mapping runtime。

---

# 32. ESDF PointCloud2 过滤

`esdf_cloud()` 只保留有限距离且满足：

```text
abs(distance) <= max_visualized_distance_m
```

默认：

```yaml
max_visualized_distance_m: 2.0
```

因此 RViz 中看到的点并不是规则查询网格的全部点，而是经过距离范围过滤后的点。

---

# 33. RViz 配置

文件：

```text
rviz/realtime_nvblox.rviz
```

默认：

```text
Fixed Frame = odom
Frame Rate  = 30
```

显示：

```text
Grid
Mesh
ESDF Slice
Camera Path
Camera Pose
```

`ESDF 3D` 默认关闭，因为密集局部 3D 点云容易让 RViz 显示变重。

---

# 34. Stats 统计系统

文件：

```text
stats.py
```

核心类：

```python
RuntimeStats
```

记录三类数据。

## 34.1 Count

例如：

```text
vio_pose
vio_failed
depth_integrated
depth_pose_drop
color_integrated
esdf_updated
mesh_updated
mesh_output
esdf_slice_output
esdf_3d_output
```

## 34.2 Rate

根据相邻 snapshot 的 count 差值和时间差计算 `rates_hz`。

## 34.3 Latency

每类保留最近 512 个耗时样本，输出：

```text
mean_ms
p50_ms
p95_ms
max_ms
```

典型 latency 项：

```text
vio_track
depth_integrate
color_integrate
esdf_update
mesh_update
mesh_to_cpu
esdf_slice_query
esdf_3d_query
```

---

# 35. 输出文件

默认目录：

```yaml
output:
  directory: ~/nvblox_output
```

生成：

```text
~/nvblox_output/realtime_map.nvblox
~/nvblox_output/realtime_mesh.ply
~/nvblox_output/runtime_stats.json
```

退出时默认保存 Map 和 Mesh。

---

# 36. config.py

负责加载 YAML，并通过 `deep_merge()` 支持：

```text
default config
+
custom config
```

因此自定义参数时无需复制整份 YAML。

例如：

```yaml
mapping:
  voxel_size_m: 0.03
```

其它参数仍继承默认配置。

---

# 37. default.yaml 参数区域

当前主要分为：

```text
frames
realsense
vslam
mapping
esdf_slice
esdf_3d
output
```

建议后续调参也按这几个逻辑模块分组。

---

# 38. runtime_loader.py

用于解决 native `.so` 冲突。

典型问题：

```text
cuVSLAM Python Binding
    ↓
依赖 libcuvslam.so

nvblox_torch Python Binding
    ↓
依赖 libnvblox_lib.so
```

如果当前 shell 之前 source 过 Isaac ROS 或其它 nvblox workspace，同名库可能被优先加载，最终出现 `undefined symbol`。

代码通过绝对路径查找 wheel 自带的：

```text
libcuvslam.so
libnvblox_lib.so
```

再用：

```python
ctypes.CDLL(..., RTLD_GLOBAL)
```

显式预加载。

---

# 39. source_realtime_nvblox.sh

仓库根目录环境脚本。

先清除：

```text
AMENT_PREFIX_PATH
COLCON_PREFIX_PATH
CMAKE_PREFIX_PATH
PYTHONPATH
ROS_DISTRO
...
```

然后 source：

```bash
/opt/ros/humble/setup.bash
```

再找到 cuVSLAM、nvblox_torch、Torch native libs，重新组织 `LD_LIBRARY_PATH`，最后加载本 workspace 的 `install/local_setup.bash`。

---

# 40. check_env.py

运行：

```bash
python3 -m realtime_nvblox.check_env
```

检查：

```text
Python
setuptools
NumPy
PyTorch
CUDA
GPU
pyrealsense2
RealSense devices
cuVSLAM
cuVSLAM v15 API
nvblox_torch
Mapper creation
```

理想结束：

```text
=== PASS ===
```

---

# 41. CLI：Standalone 模式

文件：

```text
cli.py
```

`setup.py` 注册：

```text
standalone_mapper = realtime_nvblox.cli:main
```

运行：

```bash
standalone_mapper \
  --config \
  ~/realtime_nvblox_ws/src/realtime_nvblox/config/default.yaml
```

也可以：

```bash
./scripts/run_standalone.sh
```

此模式直接打开 D435i，不需要 ROS node 和 RViz，适合先验证底层主链。

---

# 42. ROS Mapper 入口

`setup.py` 注册：

```text
ros_mapper = realtime_nvblox.ros_node:main
```

所以可运行：

```bash
ros2 run realtime_nvblox ros_mapper
```

---

# 43. 环境检查入口

还注册：

```text
check_environment = realtime_nvblox.check_env:main
```

因此可运行：

```bash
ros2 run realtime_nvblox check_environment
```

或者：

```bash
python3 -m realtime_nvblox.check_env
```

---

# 44. reconstruction.launch.py

只启动两个节点：

```text
ros_mapper
rviz2
```

默认把 `config/default.yaml` 传给 `ros_mapper`。

运行：

```bash
ros2 launch realtime_nvblox reconstruction.launch.py
```

关闭 RViz：

```bash
ros2 launch realtime_nvblox reconstruction.launch.py run_rviz:=false
```

---

# 45. package.xml

ROS2 build type：

```text
ament_python
```

运行依赖包括：

```text
rclpy
sensor_msgs
geometry_msgs
nav_msgs
std_msgs
std_srvs
visualization_msgs
tf2_ros
rviz2
```

说明该 package 本身不是 C++ nvblox package。

---

# 46. setup.py

负责：

1. 安装 Python package；
2. 安装 config、launch、RViz、README、DEPLOY；
3. 注册三个 command-line entry point。

版本：

```text
1.0.0
```

---

# 47. setup.cfg

设置 ROS2 Python executable 安装到：

```text
$base/lib/realtime_nvblox
```

使 `ros2 run` 能发现对应入口。

---

# 48. requirements.txt

列出通用 Python 依赖：

```text
numpy
scipy
PyYAML
pyrealsense2
open3d
opencv-python
einops
imageio
matplotlib
nvtx
timm
transforms3d
```

注意：

```text
torch
torchvision
nvblox_torch
cuvslam
```

没有直接写入 `requirements.txt`，而是由安装脚本按指定 wheel 版本安装。

---

# 49. install_dependencies.sh

当前安装基线：

```text
NumPy 1.26.4
PyTorch 2.9.1 + cu128
torchvision 0.24.1
nvblox_torch 0.0.10 + CUDA12 / Ubuntu22
cuVSLAM 15.0.0 + CUDA12
pyrealsense2 2.57.7.10387
```

同时限制：

```text
setuptools < 80
```

用于兼容当前 ROS2 Humble/colcon Python package 构建流程。

---

# 50. build_ros.sh

作用：

```text
source ROS2 Humble
      ↓
清理 realtime_nvblox build/install
      ↓
colcon build
```

执行：

```bash
./scripts/build_ros.sh
```

脚本明确不加载 Isaac ROS。

---

# 51. start_clean.sh

更严格的 standalone 启动方式。

使用：

```bash
env -i
```

建立几乎全新的环境，只保留 HOME、PATH、CUDA 和 wheel native libs。

适合排查：

```text
旧 ROS workspace
Isaac ROS
LD_LIBRARY_PATH
```

造成的库冲突。

---

# 52. diagnose_native_libraries.sh

专门检查：

```text
pycuvslam*.so
libcuvslam.so

libpy_nvblox.so
libnvblox_lib.so
```

并使用 `ldd` 检查最终链接。

出现：

```text
undefined symbol
```

时优先使用此脚本。

---

# 53. check_runtime.sh

快速检查 ROS 运行状态：

```text
node list
topic list
camera_pose hz
mesh hz
esdf_slice hz
esdf_3d hz
stats
```

运行：

```bash
./scripts/check_runtime.sh
```

---

# 54. 单元测试

当前两个测试：

## test_config.py

检查：

```text
voxel_size_m > 0
vslam.enable == true
emitter_flashing == true
```

## test_pose_buffer.py

构造：

```text
Pose x=0 @ t=0
Pose x=1 @ t=1s
```

查询：

```text
t=0.5s
```

验证插值得到 `x≈0.5`。

运行：

```bash
cd ~/realtime_nvblox_ws/src/realtime_nvblox
pytest -q
```

---

# 55. 推荐部署环境

根据当前代码和部署脚本，目标环境是：

```text
Ubuntu 22.04
Python 3.10
ROS2 Humble
NVIDIA GPU
CUDA 12.x compatible driver
Intel RealSense D435i
```

Standalone 重建不需要启动 ROS2 节点。

---

# 56. 首次部署

## 56.1 Clone

```bash
cd ~
git clone https://github.com/Czz1019/realtime_nvblox_ws.git
cd ~/realtime_nvblox_ws
```

## 56.2 安装 Python / GPU 依赖

```bash
cd ~/realtime_nvblox_ws/src/realtime_nvblox
chmod +x scripts/*.sh
./scripts/install_dependencies.sh
```

## 56.3 环境检查

建议新终端运行，并避免先 source Isaac ROS：

```bash
cd ~/realtime_nvblox_ws/src/realtime_nvblox
python3 -m realtime_nvblox.check_env
```

---

# 57. Standalone 运行

推荐先：

```bash
cd ~/realtime_nvblox_ws/src/realtime_nvblox
./scripts/start_clean.sh
```

或：

```bash
./scripts/run_standalone.sh
```

终端会周期输出 JSON stats。

这样可以先确认：

```text
D435i
  ↓
cuVSLAM
  ↓
Pose
  ↓
nvblox
  ↓
TSDF / ESDF / Mesh
```

主链稳定。

---

# 58. ROS2 构建

```bash
cd ~/realtime_nvblox_ws/src/realtime_nvblox
./scripts/build_ros.sh
```

然后：

```bash
source ~/realtime_nvblox_ws/source_realtime_nvblox.sh
```

---

# 59. ROS + RViz 启动

```bash
ros2 launch realtime_nvblox reconstruction.launch.py
```

---

# 60. 检查 Topic

```bash
ros2 topic list | grep realtime_nvblox
```

---

# 61. 检查运行频率

Pose：

```bash
ros2 topic hz /realtime_nvblox/camera_pose
```

ESDF 性能不要只看 PointCloud topic，应同时看：

```bash
ros2 topic echo /realtime_nvblox/stats
```

因为：

```text
esdf_updated
```

和：

```text
esdf_3d_output
```

是不同的执行阶段。

---

# 62. 默认输出目录

```bash
ls ~/nvblox_output
```

可能得到：

```text
realtime_map.nvblox
realtime_mesh.ply
runtime_stats.json
```

---

# 63. 性能诊断重点指标

`runtime_stats.json`：

```text
rates_hz.vio_pose
rates_hz.depth_integrated
rates_hz.color_integrated
rates_hz.esdf_updated
rates_hz.mesh_updated
rates_hz.mesh_output
rates_hz.esdf_slice_output
rates_hz.esdf_3d_output
```

Latency：

```text
latency.vio_track
latency.depth_integrate
latency.color_integrate
latency.esdf_update
latency.mesh_update
latency.mesh_to_cpu
latency.esdf_slice_query
latency.esdf_3d_query
```

Queue：

```text
queues
queue_drops
```

---

# 64. 如何定位瓶颈

按照：

```text
RealSense raw frames
        ↓
vio_pose
        ↓
depth_integrated
        ↓
esdf_updated
        ↓
esdf_3d_output
```

逐级观察。

如果：

```text
depth_frames      ~30 Hz
depth_integrated  ~15 Hz
```

重点排查：

```text
Depth RateLimiter
PoseBuffer 等待
Pose 匹配失败
Depth integration latency
```

如果：

```text
depth_integrated ~30 Hz
esdf_updated     ~10 Hz
```

且配置：

```yaml
esdf_update_hz: 10
```

则这是配置本身的限频。

---

# 65. 当前实现中必须理解的工程限制

## 65.1 ESDF 3D PointCloud 不是内部 EsdfLayer 原样输出

当前：

```text
内部 nvblox ESDF
       ↓
自定义规则 3D Grid
       ↓
query_esdf
       ↓
ROS PointCloud2
```

因此 3D 点数主要由：

```yaml
esdf_3d.resolution_m
size_x_m
size_y_m
size_z_m
```

决定。

## 65.2 ESDF Slice 同样是规则查询网格

当前二维 Slice：

```text
ESDF
 ↓
_grid_2d()
 ↓
query_esdf()
```

并不是原生 nvblox EsdfSlicer 输出。

## 65.3 Mapping Thread 会等待 Pose

当前：

```python
_pose_for()
```

调用：

```python
PoseBuffer.query(... wait_ms=...)
```

默认最长等待约：

```yaml
pose_wait_ms: 35.0
```

会直接影响 Mapping Loop 节拍。

## 65.4 30 Hz RateLimiter 与约 30 Hz 输入需谨慎解释

Depth 先经过：

```python
depth_lim.ready()
```

若输入就在 30 Hz 边界附近，系统调度和时间间隔抖动可能造成额外筛帧。

所以性能分析要同时看：

```text
realsense.depth_frames
depth_integrated
depth_pose_drop
queue_drops.depth
depth_integrate latency
```

## 65.5 Mapper 操作共享 RLock

Mesh CPU copy、ESDF query、save map 等都与融合互斥，重输出可能间接影响实时 mapping。

---

# 66. build / install / log 目录

当前仓库把：

```text
build/
install/
log/
```

也提交到了 GitHub。

它们属于 `colcon` 自动生成产物。源码协作通常建议加入 `.gitignore`：

```gitignore
/build/
/install/
/log/

__pycache__/
*.pyc
.pytest_cache/
```

这样能减少本机路径和构建缓存进入版本库。

---

# 67. 模块依赖关系总结

可以把代码分为四层。

## Layer 1：基础数据与工具

```text
types.py
config.py
queues.py
rate_limiter.py
stats.py
```

## Layer 2：硬件与算法 Backend

```text
runtime_loader.py
realsense_source.py
cuvslam_backend.py
pose_buffer.py
nvblox_backend.py
```

## Layer 3：实时调度

```text
runtime.py
```

## Layer 4：外部接口

```text
cli.py
ros_publishers.py
ros_node.py
launch/
rviz/
```

依赖关系：

```text
                  config.py
                     │
                     ▼
              RealtimeNvbloxRuntime
                     │
        ┌────────────┼────────────┐
        │            │            │
        ▼            ▼            ▼
RealSenseSource  CuVslamBackend  NvbloxBackend
        │            │            │
        │            ▼            │
        │        PoseBuffer       │
        │            │            │
        └────────────┴────────────┘
                     │
                     ▼
              Runtime callbacks
                /          \
               /            \
              ▼              ▼
           cli.py         ros_node.py
                             │
                             ▼
                     ros_publishers.py
                             │
                             ▼
                           RViz2
```

---

# 68. 推荐源码阅读顺序

```text
1. config/default.yaml
2. types.py
3. realsense_source.py
4. cuvslam_backend.py
5. pose_buffer.py
6. nvblox_backend.py
7. runtime.py
8. ros_publishers.py
9. ros_node.py
10. DEPLOY.md
```

其中：

> `runtime.py` 是调度核心，`realsense_source.py` 是数据入口，`cuvslam_backend.py` 是定位核心，`nvblox_backend.py` 是建图核心。

---

# 69. 一句话理解主要文件

| 文件 | 核心作用 |
|---|---|
| `types.py` | 定义系统内部数据结构 |
| `config.py` | 读取和合并 YAML |
| `queues.py` | 实现保留最新帧的有界队列 |
| `rate_limiter.py` | 控制各任务执行频率 |
| `stats.py` | 统计 count、rate 和 latency |
| `runtime_loader.py` | 解决 native `.so` 版本冲突 |
| `check_env.py` | 检查 CUDA、D435i、cuVSLAM、nvblox 环境 |
| `realsense_source.py` | 直接读取 D435i 并做 emitter 分流 |
| `cuvslam_backend.py` | 双目 + IMU VIO/SLAM |
| `pose_buffer.py` | Pose 缓存、等待和插值 |
| `nvblox_backend.py` | TSDF/ESDF/Mesh GPU 建图接口 |
| `runtime.py` | 实时系统总调度器 |
| `ros_publishers.py` | 内部数据转 ROS message |
| `ros_node.py` | ROS topic/service/TF 外壳 |
| `cli.py` | 无 ROS standalone 入口 |
| `default.yaml` | 核心参数 |
| `reconstruction.launch.py` | 启动 ros_mapper + RViz |
| `realtime_nvblox.rviz` | RViz 默认布局 |

---

# 70. 项目当前数据流总结

```text
D435i
│
├─ emitter OFF
│    ├─ IR1
│    └─ IR2
│          │
│          └─────┐
│                ▼
│              cuVSLAM
│                ▲
│                │
├────────────── IMU
│                │
│                ▼
│          T_world_rig
│                │
│          PoseBuffer
│                │
│        timestamp interpolation
│                │
├─ emitter ON    │
│    Depth ──────┼─────────────┐
│                │             │
├─ Color ────────┼──────────┐  │
│                │          │  │
│                ▼          ▼  ▼
│          T_world_color  T_world_depth
│                │          │
│                └────┬─────┘
│                     ▼
│              nvblox_torch
│                     │
│                   TSDF
│               /          \
│              ▼            ▼
│           ESDF          Mesh
│           │               │
│      ┌────┴────┐          │
│      ▼         ▼          ▼
│ 2D query   3D query   mesh_numpy
│      │         │          │
└──────┴─────────┴──────────┘
                │
                ▼
             ROS2
                │
                ▼
              RViz2
```

---

# 71. 后续架构升级时重点关注位置

如果后续要把当前 Python Query-Grid 输出改成更接近原生 nvblox / Isaac ROS 的数据流，最主要需要修改：

```text
runtime.py
    _grid_2d()
    _grid_3d()
    _output_esdf_slice()
    _output_esdf_3d()

nvblox_backend.py
    query_esdf()
```

而以下部分仍可复用：

```text
RealSenseSource
EmitterClassifier
CuVslamBackend
Calibration
PoseBuffer 思路
Stats
配置系统
ROS 输出层的部分接口
```

如果进一步改成直接调用 `nvidia-isaac/nvblox` C++ Core，则 Mapping Backend 与 ESDF Output 层需要重新组织。

---

# 72. License

当前 package 使用：

```text
MIT
```

见：

```text
LICENSE
package.xml
setup.py
```

---

# 73. 推荐调试顺序

```text
1. Python环境
      ↓
2. D435i
      ↓
3. cuVSLAM
      ↓
4. PoseBuffer
      ↓
5. Depth integration
      ↓
6. ESDF update
      ↓
7. Mesh update
      ↓
8. Standalone稳定性
      ↓
9. ROS2
      ↓
10. RViz
      ↓
11. ESDF 2D/3D输出
```

不要一开始就只通过 RViz 是否流畅判断底层 mapping 是否实时。

---

# 74. 常用命令速查

环境：

```bash
source ~/realtime_nvblox_ws/source_realtime_nvblox.sh
```

检查：

```bash
python3 -m realtime_nvblox.check_env
```

Standalone：

```bash
cd ~/realtime_nvblox_ws/src/realtime_nvblox
./scripts/start_clean.sh
```

ROS build：

```bash
./scripts/build_ros.sh
```

ROS + RViz：

```bash
ros2 launch realtime_nvblox reconstruction.launch.py
```

Runtime 检查：

```bash
./scripts/check_runtime.sh
```

保存 Map：

```bash
ros2 service call \
  /realtime_nvblox/save_map \
  std_srvs/srv/Trigger '{}'
```

保存 Mesh：

```bash
ros2 service call \
  /realtime_nvblox/save_mesh \
  std_srvs/srv/Trigger '{}'
```

查看统计：

```bash
ros2 topic echo /realtime_nvblox/stats
```

GPU：

```bash
watch -n 1 nvidia-smi
```

---

# 总结

`realtime_nvblox_ws` 当前是一个完整的：

```text
RealSense D435i
      +
Stereo/IMU cuVSLAM
      +
Pose timestamp alignment
      +
nvblox_torch GPU mapping
      +
ROS2/RViz visualization
```

实验工程。

最重要的三个设计点是：

1. **直接用 pyrealsense2 和 emitter 状态将 IR/VSLAM 与 Depth/Mapping 分离；**
2. **通过 PoseBuffer 将不同采集时刻的数据对齐到统一连续 VIO 坐标系；**
3. **把 ROS2 放在核心 Runtime 外部，使建图链可以独立于 Isaac ROS 与 ROS topic 运行。**

同时当前 ESDF 可视化需要明确理解为：

```text
nvblox 内部 ESDF
       ↓
自定义固定分辨率查询网格
       ↓
ROS PointCloud2
```

因此 **ESDF 内部地图分辨率** 和 **ROS 中看到的 ESDF 点云采样分辨率** 不是同一个参数。后续若要进一步贴近原生 nvblox / Isaac ROS 数据流，这一层是最需要重构的位置。
