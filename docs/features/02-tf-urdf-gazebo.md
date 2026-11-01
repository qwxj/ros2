# TF2 / URDF / Gazebo 仿真

> 本文是 README「已实现功能」的拆分子集，收录 **§10 ~ §12**（3 个小节）。
> 原始完整文档： [`docs/archive/README-2026-11-01-full.md`](../archive/README-2026-11-01-full.md) 第 233~271 行。

### 10. TF2 坐标变换


- 使用 `TransformBroadcaster` 发布坐标系之间的变换关系。
- 使用 `TransformListener`、`Buffer` 和 `lookupTransform` 查询坐标变换。
- 模拟里程计数据，广播 `odom → base_link` 坐标变换。
- 使用 `tf2_geometry_msgs` 实现不同坐标系下的点坐标转换。
- 实践四元数与欧拉角转换、四元数乘法等姿态表示基础。
- 实现激光雷达点数据发布，以及从 `laser` 坐标系到 `base_link` 坐标系的点变换。

> **涉及文件**（`src/my_first_pkg/src/`）：`tf2_broadcaster_demo.cpp`、`tf2_listener_demo.cpp`、
> `tf2_odom_broadcaster_demo.cpp`、`tf2_point_transform_demo.cpp`、`quaternion_demo.cpp`、
> `laser_point_publisher.cpp`、`laser_point_transformer.cpp`

### 11. URDF 机器人模型与 RViz2 可视化


- 新建 `my_robot_description` 功能包，用于存放机器人模型与可视化配置。
- 手写两轮差速小车的 URDF：`base_footprint` → `base_link` 固定关节，左右轮为 `continuous` 关节。
- 通过 `robot_state_publisher` 将 URDF 转换为 TF，并在 RViz2 中显示模型与坐标系。
- 使用 `joint_state_publisher_gui` 拖动滑块驱动轮子关节，观察关节角度与 TF 的联动。
- RViz 配置以文件形式随包安装（`rviz/view.rviz`），launch 时通过 `-d` 加载，无需每次手动配置显示项。

> **涉及文件**：`src/my_robot_description/urdf/robot.urdf`、`src/my_robot_description/launch/display.launch.py`、
> `src/my_robot_description/rviz/view.rviz`、`src/my_robot_description/CMakeLists.txt`、`src/my_robot_description/package.xml`

### 12. Gazebo 物理仿真与差速驱动


- 为 URDF 补齐物理属性：每个 link 加 `<inertial>`（质量 + 惯性张量）与 `<collision>`，
  否则模型在 Gazebo 里没有质量、会直接穿过地面。
- 前后各加一个**万向轮（caster）**，解决"两个轮子撑不住车体、一落地就前后翻"的问题；
  并把 caster 的摩擦系数设为 0，避免固定球体在地上"刮"住车。
- 使用 `libgazebo_ros_diff_drive.so` 实现差速驱动：订阅 `/cmd_vel`、发布 `/odom` 与
  `odom → base_footprint` 变换，并直接发布轮子 TF（因此不需要 `joint_state_publisher`）。
- 用 `.world`（SDF）文件搭建可复现的测试场景：地面 + 三个障碍物（方块 / 圆柱 / 墙）。
- `launch/gazebo.launch.py` 用 `IncludeLaunchDescription` + `launch_arguments` 传入 world 文件
  并生成模型；`use_sim_time: true` 让 TF 时间戳跟随仿真时钟。

> **涉及文件**：`src/my_robot_description/urdf/robot.urdf`、`src/my_robot_description/launch/gazebo.launch.py`、
> `src/my_robot_description/worlds/my_world.world`
