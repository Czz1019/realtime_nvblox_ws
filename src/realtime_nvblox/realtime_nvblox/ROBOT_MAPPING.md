# 机械臂位姿建图

使用 `robot_eye_in_hand.yaml`，它覆盖同目录 `default.yaml`。原 cuVSLAM 默认配置保留。
已填入提供的 `base_link`、`tool0` 和手眼标定：平移单位为米，四元数为 xyzw。
`camera_marker` 仅用于标定过程，运行时不需要跟踪标记。

## 运行源码

当前工作空间 install 目录是旧代码副本。以下命令显式选择修改后的源码，避免误运行旧安装：

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=/home/czz/realtime_nvblox_ws/src/realtime_nvblox:${PYTHONPATH:-}
/usr/bin/python3 -m realtime_nvblox.ros_node --ros-args \
  -p config:=/home/czz/realtime_nvblox_ws/src/realtime_nvblox/realtime_nvblox/robot_eye_in_hand.yaml
```

先启动机械臂状态发布节点，确保 TF 中存在动态的 `base_link → tool0`（可以经中间关节）。
本节点不会发送机械臂运动指令。RealSense 由本节点打开，不能同时被另一采集节点占用。
默认不广播相机 TF，避免与机械臂/相机驱动的已有 TF 树冲突。

相机使用 RealSense host/global time；机械臂必须发布同一系统时间轴上的采样时间，
多机部署须同步时钟，不能以回调到达时间代替采样时间，不能使用 `use_sim_time`。
时间域未就绪的相机帧会丢弃并计数，持续未就绪会停止运行。

## 变换约定

`T_A_B` 将 B 坐标变换到 A。配置中的手眼外参为 `T_tool_color`：

```text
T_base_depth(t) = T_base_tool(t)
                  × T_tool_color
                  × inverse(T_rig_color)
                  × T_rig_depth
```

左右/深度/彩色相机间外参取自 RealSense 标定，按列主序读取旋转数组。
默认 world 就是 base_link；如果改为其他 world，必须显式提供 `robot_pose.T_world_base`。
`/realtime_nvblox/camera_pose` 表示左红外光学 frame（rig）在 world 中的位姿，融合使用转换后的 depth 位姿。

TF 模式在深度帧时间查询 tool0，不退回 latest TF，也不外推。缺少对应时刻 TF 的帧会丢弃。
TF2 自身负责插值；`max_pose_error_ms` 和 `allow_nearest` 只用于下面的话题模式。

如果没有 TF，可将 `robot_pose.input` 改为 `topic`，填写真实 `topic`，并选择
`TransformStamped` 或 `PoseStamped`。前者校验父/子 frame；后者只含父 frame，
因此话题内容必须确实表示 tool0。话题模式先插值末端位姿再乘手眼外参，默认禁止最近邻回退。

## ESDF 调度与统计

机械臂配置使用连续 30 FPS 深度，关闭交替激光和 VIO/IMU 采集，`depth_hz: 0` 表示不重复筛帧。
地图有新融合数据时才更新 ESDF；未到更新时刻会保留待更新状态。
3D 查询保留尚未到期的请求，到期后选最新地图；地图版本、时间戳和查询由同一 Mapper 锁保护。
ROS 只发布新结果，不通过重复旧地图补足频率。

网格是围绕中心对称、间距精确且不超出指定范围的采样点集合。
4×4×2 m、0.15 m 配置现在为 27×27×14 = 10,206 个点；非整除尺寸不会强行覆盖两端。
`center_on_camera: true` 时，偏移为 `[center_x_m, center_y_m, z_offset_m]`，
`offset_frame: world` 沿世界轴偏移，`camera` 沿相机光学轴偏移；`center_z_m` 仅用于固定中心模式。

观察 `/realtime_nvblox/stats` 中：

- `rates_hz.depth_integrated / esdf_updated / esdf_3d_generated / esdf_3d_published`：逐级实际频率。
- `depth_pose_drop / robot_tf_unavailable / depth_rate_skipped`：位姿不足或限流丢帧。
- `latency.pose_wait / depth_age / esdf_3d_age`：位姿等待、融合和发布时的数据年龄。
- `latency.esdf_3d_lock_wait / esdf_3d_query_submit / esdf_3d_readback`：查询等待、调用和回传耗时。
- `esdf_3d.latest_version`：生成结果的地图版本，应持续增长。

`query_submit` 是主机端调用耗时，`readback` 包含 GPU 等待，不等同于纯 kernel 计时。
`pose_time_error` 在话题模式中为最近邻误差或插值两侧最大时间间隔；TF 模式不提供插值端点误差。

## 测试与实机验收

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=/home/czz/realtime_nvblox_ws/src/realtime_nvblox:${PYTHONPATH:-}
cd /home/czz/realtime_nvblox_ws/src/realtime_nvblox/realtime_nvblox
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 -m pytest -q -p no:cacheprovider tests ../test
# 有 CUDA 时额外执行：
RUN_GPU_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 -m pytest -q -p no:cacheprovider tests/test_gpu_esdf.py
```

关闭 pytest 第三方插件自动加载用于避开本机已有的 typeguard 插件冲突。
测试覆盖末端旋转偏置、真实标定转换、TF 曝光时刻插值、限流边界/抖动、待处理请求、
连续深度采集、网格边界以及 GPU 深度融合/查询与缓冲区所有权。

实机需验证：静止物体在机械臂平移/旋转时仍重合；地图版本持续增加；
各级频率接近目标且数据年龄不持续增长。当前没有连接 RealSense，尚未完成这部分验收。
