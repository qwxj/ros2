# ROS 2 学习与开发实践

基于 **ROS 2 Humble** 和 **C++** 搭建的个人学习与实践工作区。围绕 ROS 2 常用通信机制、节点管理、并发执行、进程内零拷贝通信、参数配置、TF2 坐标变换及 URDF 机器人建模等内容，编写了多个可独立编译、运行的示例程序，用于理解 ROS 2 的核心开发模型。

本项目以功能实现和代码实践为主，持续补充机器人软件开发相关内容。

## 项目简介

- **项目类型：** 个人学习与实践项目
- **开发语言：** C++
- **中间件：** ROS 2 Humble（默认 RMW：Fast DDS）
- **开发平台：** Ubuntu 22.04
- **构建工具：** CMake、colcon
- **代码组织：** 3 个 ROS 2 功能包，37 个 C++ 源文件（35 个可独立运行的 Demo + 2 个组件）

## 技术栈与开发环境

| 项目 | 环境 / 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble |
| 中间件 | Fast DDS（含 `DEFAULT_FASTDDS_PROFILES.xml` 自定义 profile） |
| 编译器 | GCC / G++ 11.4.0 |
| 构建工具 | CMake 3.22.1、colcon |
| 开发语言 | C++ |
| 可视化 | RViz2 |

## 已实现功能

> **路径约定**：下面「涉及文件」里的路径都相对于仓库根目录。为简洁，`src/my_first_pkg/src/` 与
> `src/my_robot_interfaces/` 这种公共前缀在条目里只写一次，后面直接列文件名。

### 1. Topic 话题通信

- 实现 Publisher / Subscriber 发布订阅模型。
- 使用定时器周期性发布消息，订阅端接收并处理消息。
- 通过 QoS 配置示例，实践 RELIABLE、BEST_EFFORT 等可靠性策略及历史队列深度。
- 使用 `rclcpp_components` 将节点封装为组件，了解组件化节点的组织方式。

> **涉及文件**（`src/my_first_pkg/src/`）：`my_publisher.cpp`、`my_subscriber.cpp`、`qos_publisher.cpp`、`topic_components.cpp`

### 2. Service 服务通信与自定义接口

- 基于 `example_interfaces/srv/AddTwoInts` 实现服务端和客户端。
- 创建自定义消息 `DeviceStatus`，用于描述设备名称、在线状态和温度信息。
- 创建自定义服务 `GetDeviceStatus`，根据设备名称返回设备状态信息。
- 实践自定义接口包的定义、编译及在其他功能包中的调用。

> **涉及文件**：`src/my_first_pkg/src/` 下 `add_server.cpp`、`add_client.cpp`、`device_status_publisher.cpp`、`device_status_subscriber.cpp`、`get_device_status_server.cpp`、`get_device_status_client.cpp`；
> 接口定义在 `src/my_robot_interfaces/msg/DeviceStatus.msg`、`src/my_robot_interfaces/srv/GetDeviceStatus.srv`

### 3. Action 动作通信

- 使用 `rclcpp_action` 实现 Action Server 和 Action Client。
- 实践长时间任务的异步执行、过程反馈及任务取消。
- 理解 Topic、Service 与 Action 在通信场景上的区别。

> **涉及文件**（`src/my_first_pkg/src/`）：`action_server.cpp`、`action_client.cpp`（基于 `example_interfaces/action/Fibonacci`）

### 4. 节点生命周期与组件

- 编写 Lifecycle Node，实践 configure、activate、deactivate、cleanup、shutdown 等状态转换。
- 实现普通节点及其参数、定时器功能。
- 将节点封装为 `rclcpp_components` 组件，了解组件注册与加载方式。

> **涉及文件**（`src/my_first_pkg/src/`）：`lifecycle_demo.cpp`、`status_node.cpp`、`status_component.cpp`（后两个是"同一个节点的普通版 / 组件版"对照）

### 5. Executor 与回调组并发控制

- 使用 `SingleThreadedExecutor` 实践单线程回调调度。
- 使用 `MultiThreadedExecutor` 实践多线程回调调度。
- 通过多个定时器的执行示例，观察回调执行与阻塞现象，理解 ROS 2 执行器的基本工作方式。
- 使用 `MutuallyExclusive` 与 `Reentrant` 两种 Callback Group，对比同一回调组内串行执行与并发执行的区别。
- 通过多回调组 + 多节点组合，实践**任务隔离**：把耗时回调与实时性要求高的回调放进不同组，避免相互阻塞。
- 用独立工作线程承接耗时操作，把回调本身保持轻量，避免拖慢执行器。

> **涉及文件**（`src/my_first_pkg/src/`）：`executor_single_demo.cpp`、`executor_multi_demo.cpp`、`executor_demo.cpp`、`executor_callback_group_demo.cpp`、`task_isolation_demo.cpp`、`worker_queue_demo.cpp`

### 6. 进程内通信与零拷贝（Loaned Message）

- 使用 `borrow_loaned_message` / `publish_loaned` 实现**零拷贝**消息发布，跑通 loaned message 完整链路。
- 通过 `can_borrow` 判断当前 RMW 是否支持 loaned message，并打印结果。
- 编写 `DEFAULT_FASTDDS_PROFILES.xml`，为 `/loaned_topic` 单独配置 `data_sharing`：
  默认 profile 使用 `OFF`，`/loaned_topic` 使用 `AUTOMATIC`，从而启用 Fast DDS 的数据共享（共享内存）通路。
- 对比**普通发布**与 **loaned 发布**的端到端延迟，量化零拷贝带来的收益。
- 实测结论：1 MB 消息、10 Hz 条件下，loaned 方式相比普通方式约有 **30%** 的延迟下降。

> **涉及文件**：`src/my_first_pkg/src/` 下 `loaned_message_demo.cpp`、`loaned_compare.cpp`、`big_loaned_compare.cpp`；
> 接口 `src/my_robot_interfaces/msg/LoanedData.msg`、`src/my_robot_interfaces/msg/BigLoanedData.msg`；
> 配置 `DEFAULT_FASTDDS_PROFILES.xml`（仓库根目录）

### 7. 大块数据传输与内存池复用

- 定义两类大消息接口：
  - `BigData`：`uint8[] payload`（**变长**，每次发布都要分配）；
  - `BigLoanedData`：`uint8[1048576] payload`（**定长 1 MB**，可用于 loaned message）。
- 对比变长消息与定长消息在不同通信方式下的表现。
- 引入**对象池 / 内存池**复用缓冲区，避免每帧重复分配释放，对比复用前后的开销。
- 提供两种发送策略用于对照：
  - `bigdata_pool_benchmark`：正常入队策略；
  - `bigdata_pool_benchmark_loss`：**允许丢弃**（队列满即丢），避免发布端被慢订阅者拖住。

> **涉及文件**：`src/my_first_pkg/src/` 下 `bigdata_benchmark.cpp`、`bigdata_pool_benchmark.cpp`、`bigdata_pool_benchmark_loss.cpp`；
> 接口 `src/my_robot_interfaces/msg/BigData.msg`、`src/my_robot_interfaces/msg/BigLoanedData.msg`

### 8. 通信延迟测量与问题排查

- 编写 `LatencyData` 消息，携带序号与发送时间戳，用于计算端到端延迟。
- `latency_publisher` / `latency_subscriber` 组成最小延迟测量链路。
- `latency_subscriber` 提供 `callback_mode` 参数，可在 `mutually`（默认）与 `reentrant` 两种回调组之间切换，
  用于观察**回调并发方式对测量结果的影响**。
- `bigdata_benchmark` 支持切换通信通路做对照测试：`intra`（进程内）、`dds`（走 DDS）、`ipc_pub` / `ipc_sub`（跨进程）。

> **涉及文件**：`src/my_first_pkg/src/` 下 `latency_publisher.cpp`、`latency_subscriber.cpp`、`bigdata_benchmark.cpp`；
> 接口 `src/my_robot_interfaces/msg/LatencyData.msg`

### 9. 参数管理

- 使用 `declare_parameter` 声明参数并读取参数值。
- 通过命令行和 YAML 文件传递节点参数。
- 注册参数变更回调，实践运行过程中的动态参数修改。
- `image_processor` 把 Canny 双阈值做成参数，注册 `add_on_set_parameters_callback` 响应运行时
  `ros2 param set`，并做范围校验（`0 <= canny_low < canny_high <= 255`），非法值直接拒绝。
- 参数持久化：写进 `config/image_processor.yaml`（必须带节点名 + `ros__parameters`），
  通过 launch 的 `parameters=[yaml_path]` 或命令行 `--params-file` 加载。
  优先级为 **默认值 < YAML < 运行时 `param set`**（后者重启即失效）。

> **涉及文件**（`src/my_first_pkg/src/`）：`parameter_demo.cpp`（参数 `speed`）、`status_node.cpp`（参数 `interval_ms`）；
> 参数文件 `src/my_first_pkg/config/parameter_demo.yaml`、`src/my_first_pkg/config/params.yaml`、
> `src/my_first_pkg/config/image_processor.yaml`；参数化节点 `src/my_first_pkg/src/image_processor.cpp`

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

### 13. 速度指令看门狗（cmd_vel_watchdog）

- `/cmd_vel` 是**设定值**而非一次性命令：差速驱动插件会把最后收到的速度一直保持下去，
  遥控端掉线时机器人会一直冲（Gazebo 的差速插件本身**没有任何超时机制**）。
- 自写节点 `cmd_vel_watchdog`：把 `/cmd_vel` 转发到 `/cmd_vel_safe`；
  若 `timeout_sec` 内没有新指令，就主动发布**全零**（注意不是"停止发布"——停止发不会刹车）。
- 超时用 `std::chrono::steady_clock` 而不是 `this->now()`：Gazebo 一旦暂停，ROS 时间会冻住，
  用仿真时间会让看门狗**彻底失效且不报错**。
- 超时阈值必须**大于**指令发布周期（`τ ≥ 2~3 × T`），否则每个周期都会误报，
  机器人表现为"走一下、停一下"。
- `timeout_sec` 做成参数，可用 `ros2 param set` 运行时调整，直观体会"安全性 vs 可用性"的取舍。

- **架构演进**：最初看门狗是「旁路」形态（`/cmd_vel_input` → `/cmd_vel`），接入 Nav2 后与
  `velocity_smoother` 抢同一个话题，导致车一顿一顿；现改为**串联安全层**，详见 §19。

> **涉及文件**：`src/my_first_pkg/src/cmd_vel_watchdog.cpp`

### 14. 激光雷达与仿真环境

- 在车顶加 `lidar_link`（fixed 关节），用 `<sensor type="ray">` + `libgazebo_ros_ray_sensor.so`
  输出二维扫描；`output_type` 必须显式写 `sensor_msgs/LaserScan`（不写默认是 `PointCloud2`）。
- 通过 `~/out:=scan` 重映射把话题改成 `/scan`，用 `frame_name` 指定 `frame_id`。
- 扫描参数：360 线 × 整圈（±π）、量程 0.12~8 m，并叠加 σ=1 cm 高斯噪声以贴近真实雷达。
- 实测 `/scan` 约 9.9 Hz（配置 10 Hz）；`angle_increment` 为 0.0175 rad ——
  Gazebo 用的是 `(max-min)/(samples-1)`，与 ROS 惯例的 `/samples` 不同，读数要以消息字段为准。
- 空世界里雷达全是 `inf`（水平扫描打不到地面）；加入障碍物后测距值与几何尺寸吻合
  （墙在 x=-2.0 时读数约 1.94~2.07 m）。

> **涉及文件**：`src/my_robot_description/urdf/robot.urdf`、`src/my_robot_description/worlds/my_world.world`

### 15. 相机与 cv_bridge 图像处理

- 车头加 `camera_link`，用 `<sensor type="camera">` + `libgazebo_ros_camera.so`
  输出 640×480、`rgb8`、15 fps。
- 相机插件的话题命名规则是 `/<namespace>/<camera_name>/image_raw`，
  `namespace` 与 sensor 的 `name` 会**叠加** —— 两者都叫 `camera` 会得到 `/camera/camera/image_raw`，
  只保留 sensor name 才是 `/camera/image_raw`。
- 图像处理节点 `image_processor`：用 `cv_bridge::toCvCopy(msg, "bgr8")` 把 ROS 图像转成 `cv::Mat`
  （同时把 `rgb8` 转成 OpenCV 习惯的 BGR），再做灰度化 + Canny 边缘检测。
- 反向打包用 `cv_bridge::CvImage`：必须**继承原消息的 `header`**（时间戳 + frame_id），
  `encoding` 要设成 `mono8`（单通道）；发布时需 `publish(*out_msg.toImageMsg())` 解引用，
  因为 `Publisher::publish()` 没有接受 `shared_ptr` 的重载。
- 在 RViz2 用 `Image` 显示项验证：绿方块占 181×180 像素，Canny 边缘 bbox 与之完全重合，
  边缘像素 724 ≈ 矩形周长 2×(181+180)=722。

> **涉及文件**：`src/my_first_pkg/src/image_processor.cpp`、`src/my_first_pkg/config/image_processor.yaml`、
> `src/my_first_pkg/launch/image_processor.launch.py`

### 16. Launch 启动管理

- 使用 Python 编写 launch 文件。
- 实践多个节点的统一启动、参数传递及话题重映射。
- 通过 launch 文件组织多个节点的运行流程。

> **涉及文件**：`src/my_first_pkg/launch/demo.launch.py`（启动 `my_publisher`、`my_subscriber`、`parameter_demo`，加载 `parameter_demo.yaml`）、
> `src/my_first_pkg/launch/multi_node.launch.py`（启动 `my_publisher`、`status_node`，加载 `params.yaml`）、
> `src/my_robot_description/launch/display.launch.py`（启动 `robot_state_publisher` + RViz2）

### 17. 数据录制与离线回放（rosbag2）

- 用 `ros2 bag record` **显式列出话题**录制（不用 `-a`），产出两个数据集：
  `bags/drive_01`（4 话题 / 37 s / 7436 条 / 3.7 MiB）与
  `bags/with_cam`（`/camera/image_raw` + `/scan` / 19 s / 442 条 / 234 MiB）。
- 实测各话题数据量差异极大：`/scan` 9.5 Hz、`/odom` 47 Hz、
  **`/tf` 143 Hz（消息数比 `/scan` 大 15 倍）**、`/camera/image_raw` 14 Hz
  （单帧 640×480×3 ≈ 0.9 MB，10 Hz 即 9 MB/s，20 秒约 180 MB）。
  所以录制必须显式选话题 + 控制时长，盲目 `-a` 会让 bag 迅速膨胀。
- 回放用 `ros2 bag play --clock 1000`，由**播放器自己产生 `/clock`**；
  因此录制时**不需要录 `/clock`**（它 1000 Hz，录进去纯属浪费），
  而 `use_sim_time` 的节点在没有 Gazebo 时也能正常工作。
- 验证了**数据与算法解耦**：Ctrl+C 关掉 Gazebo 后回放 `bags/with_cam`，
  `image_processor` 照常处理并发布 `/image_processed`，RViz 正常显示边缘图 ——
  调算法不必再启动仿真、开车对准目标，且每次输入完全一致（可复现）。
- 常用参数：`-r 0.5` 慢放、`-l` 循环、`--topics` 只放部分话题、
  `-p` 启动即暂停、`--start-offset` 跳过开头、`--max-bag-duration` 限制录制时长。

> **涉及目录**：`bags/`（录制数据，已在 `.gitignore` 中排除，不入库）

### 18. SLAM 建图（slam_toolbox）

- 用 `slam_toolbox` 跑通「激光扫描 + 里程计 → 栅格地图」的完整链路：
  实时开车用 `online_async_launch.py`（异步、不阻塞），喂 bag 用 `online_sync_launch.py`。
- SLAM 在 TF 树里**只负责发布一段 `map → odom`**：通过扫描匹配估计机器人在 `map` 中的位姿，
  再把 `odom` 的累积漂移补偿掉。另外两段由别人提供 ——
  `odom → base_footprint`（差速插件）、`base_footprint → base_link → lidar_link`（robot_state_publisher）。
- 关键参数：`base_frame: base_footprint`、`scan_topic: /scan`、`resolution: 0.05`、
  `minimum_travel_distance / heading`（走多远、转多少插一个节点，仿真小车调到 0.2）、
  `map_update_interval`（地图刷新周期）、`min/max_laser_range`（与 URDF 量程一致）。
- 建图必须**真的移动**：原地不动只有一帧扫描，地图长不出来；
  绕一圈回到起点会触发**闭环检测（loop closure）**，能看到地图自动对齐修正。
- 成果 `maps/my_map.pgm` + `my_map.yaml`：110×81 px @ 0.05 m/px = **5.5 m × 4.05 m**，
  空闲 76.9% / 障碍 2.4% / 未知 20.7%，`origin` 与 world 中障碍物位置吻合。

> **涉及文件**：`src/my_robot_description/config/slam_toolbox.yaml`、
> `maps/my_map.pgm`、`maps/my_map.yaml`

### 19. Nav2 自主导航

- 用 `nav2_bringup/bringup_launch.py` 拉起完整导航栈：`map_server` + `amcl` + `planner_server` +
  `controller_server` + `behavior_server` + `bt_navigator` + `velocity_smoother`，
  由 `lifecycle_manager` 按依赖顺序自动 configure/activate（并靠 bond 监控存活）。
- **导航是动作**：`NavigateToPose`（`nav2_msgs/action`）—— 有反馈（`distance_remaining`）、
  可取消，正是 §3 Action 在真实系统里的用法。
- 地图与定位分工：`map_server` 只把 `my_map.yaml` 作为 `/map` **话题**发出来；
  AMCL 负责在已知地图上定位，并**发布 `map → odom`** —— 与 slam_toolbox 互斥，同一时间只能有一个。
- 初始位姿必须显式给出（RViz 的 `2D Pose Estimate`，或 `ros2 topic pub /initialpose`），
  否则 AMCL 不发 `map → odom`，TF 树里就没有 `map` 这个坐标，所有代价地图会一直刷
  `Timed out waiting for transform ... frame does not exist`。
- 按 0.4×0.3 m 的小车改了 6 处参数：`laser_max_range 8.0`、`max_particles 1000`、
  两个 costmap 的 `robot_radius 0.30`（= √(0.235²+0.18²) 的外接圆）、
  `max_vel_x / max_speed_xy 0.15`。
- **架构修复：把看门狗从「旁路」改成「串联安全层」**。原先 watchdog 和 Nav2 都在往
  `/cmd_vel` 发，互相打架；现在：

  ```
  Nav2(velocity_smoother) ──┐
                            ├──▶ /cmd_vel ──▶ watchdog ──▶ /cmd_vel_safe ──▶ 差速插件
  teleop / 手动 pub ────────┘
  ```

  差速插件通过 URDF 的 `<ros><remapping>cmd_vel:=cmd_vel_safe</remapping>` 改听话的话题。
  效果是所有下发底盘的速度**必须**经过看门狗 —— 任何指令源断流，0.5 秒后自动停车
  （Nav2 进程崩溃也一样）。验证方法：持续发速度 → 掐掉发布进程 → 车应自动停。

> **涉及文件**：`src/my_robot_description/config/nav2_params.yaml`、
> `src/my_first_pkg/src/cmd_vel_watchdog.cpp`、`src/my_robot_description/urdf/robot.urdf`、
> `maps/my_map.yaml`

### 20. Nav2 参数调优：离线实验方法

- **要解决的问题**：在线调参时每次都要开 Gazebo、开车、复位，输入条件一直在变 ——
  改了参数后车走得好了，也说不清是参数起了作用还是这次起点恰好更准。
- **方法**：录一段**车全程静止**的 bag 作为固定输入，离线回放给 Nav2；
  位姿恒定，**只改一个参数**，对比规划器输出的 `/plan`。
- **关键认知：离线回放是"开环"的**。bag 里的 `/odom`、`/tf` 决定机器人位姿，
  Nav2 输出的 `/cmd_vel` **不会**驱动机器人（没有物理引擎消费它）。
  所以离线能对比的是「**决策**」（`/plan`、`/local_plan`、`/cmd_vel` 数值），
  不能对比「**结果**」（到达时间、实际行驶轨迹 —— 那些完全由 bag 决定）。
- **实验台**：`bags/tune_idle`，187 s，车全程静止（`/odom` 位移跨度 **0.0008 m**、
  yaw 跨度 **0.03°**），含 `/scan` 1857 帧 · `/odom` 9285 条 · `/tf` 27855 条 · `/tf_static` 1 条。
- **度量工具**：`tools/bag_metrics.py` 直接读 `.db3`，输出全局路径长度/点数、速度统计、
  最近障碍距离，支持多个 bag 并排对比 —— **调参不能靠"看着顺眼"，要有数字**。

> **涉及文件**：`tools/bag_metrics.py`、`src/my_robot_description/config/nav2_params.yaml`

#### 实验一：`inflation_radius` 对全局路径的影响

目标点选 **(2.8, 0.0)**：从原点 (0,0) 出发的直线**必须穿过方块 (2.0, 0.0)**
（方块占 x∈[1.75, 2.25]、y∈[-0.25, 0.25]），因此必然绕行。该点距最近障碍 0.50 m、直线距离 2.80 m。

| `inflation_radius` | 路径长度 | 路径点数 | 相对直线 2.80 m |
|---|---|---|---|
| **0.55**（Nav2 默认） | **3.77 m** | 150 | +0.97 m（**+35%**） |
| **0.35**（改后） | **3.44 m** | 138 | +0.64 m（+23%） |
| **差异** | **−0.33 m（−8.8%）** | −12 | −0.33 m |

**结论**：膨胀半径从 0.55 降到 0.35，绕障路径缩短 0.33 m。
在 5.5×4 m 的房间 + 0.4×0.3 m 小车这个尺度下，0.55（= 车半径 0.30 + 0.25 余量）确实偏保守。

#### 对照：目标点选错，就测不出差异

把目标换成 **(1.0, -1.0)**（这条线上没有任何障碍物），在 `inflation_radius: 0.55` 下：

| 项 | 值 |
|---|---|
| 路径长度 | **1.43 m** |
| 直线距离 | 1.414 m |
| 差值 | **+1.6 cm（+1.1%）** |

同样的参数、同样的小车，换个目标点，影响从 **+35%** 掉到 **+1.1%** ——
说明**膨胀半径只在路径经过障碍物附近时才起作用**。
实验设计（目标点怎么选）比参数本身更影响结论的可信度。

#### 实验流程

```bash
# ① 录制固定输入（Gazebo 中，车全程静止不动）
ros2 bag record -o bags/tune_idle /scan /odom /tf /tf_static

# ② 改参数后重启 Nav2（参数只在启动时读取，不用 colcon build）
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/ros2/maps/my_map.yaml \
  params_file:=$HOME/ros2/src/my_robot_description/config/nav2_params.yaml \
  use_sim_time:=true use_composition:=False

# ③ 验证参数真的生效（插件参数必须带插件名前缀）
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius

# ④ 回放固定输入
ros2 bag play bags/tune_idle --clock 1000

# ⑤ 录制决策输出
ros2 bag record -o bags/plan_inf035 /plan /cmd_vel /local_plan

# ⑥ 发同一个目标点
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {header: {frame_id: map},
         pose: {position: {x: 2.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
}" --feedback

# ⑦ 并排对比
python3 tools/bag_metrics.py bags/base_2p8 bags/plan_inf035
```

**过程中踩到的两个坑**：

1. **启动顺序错了，Nav2 会"假装启动"然后卡死**：没有传感器数据时启动，`controller_server`
   会卡在 `local_costmap.activate()` 的 `waitForTransform` 循环里，连 `ros2 lifecycle get`
   都不返回；后续的 `planner_server` 等全部停在 `inactive`。
   必须**先让 `/scan`、`/odom` 流动起来，再启动 Nav2**。
2. **AMCL 的 TF 是"激光驱动"的**：只发 `/initialpose`、没有 `/scan` 时，AMCL 不会发布
   `map → odom`。改用 `amcl.set_initial_pose: true` + `initial_pose: {x,y,z,yaw}` 后，
   AMCL 激活即采用固定位姿，收到第一帧扫描就发 TF，省掉了"趁启动时手发初始位姿"的时序操作。

### 21. 在线控制参数调优与任务级编排

#### 速度链路上的两级限幅

Nav2 的速度指令要经过两道闸才到 `/cmd_vel`：

```
controller_server (FollowPath.max_vel_x)   ← 控制器「想发」多少
        ↓ /cmd_vel_nav
velocity_smoother (max_velocity)           ← 真正「允许」多少
        ↓ /cmd_vel
cmd_vel_watchdog → /cmd_vel_safe → 差速插件
```

**只改上游不改下游，速度会被静默削掉。** 实测：`max_vel_x: 0.30` 而
`max_velocity: [0.26, 0, 1.0]` 时，`/cmd_vel` 最大只有 **0.260**。

#### 实验二：控制参数对导航时长的影响（在线，目标点 (1.5, -1.2)，直线 1.92 m）

| 组 | `max_vel_x` | `max_vel_theta` | `smoother.max_velocity` | `\|v\|` 最大 | 运动时长 | 原地转占比 |
|---|---|---|---|---|---|---|
| A | 0.15 | 1.0 | [0.26, 0, 1.0] | 0.150 | 20.6 s | 11.2% |
| B | 0.30 | 1.0 | [0.26, 0, 1.0] | **0.260** | 17.8 s | 23.0% |
| C | 0.30 | 1.5 | **[0.30, 0, 1.5]** | **0.300** | **15.4 s** | 21.0% |

**三条结论**：

1. **B 组暴露了限幅层**：`max_vel_x` 提到 0.30，实测只到 0.260 —— `velocity_smoother` 是瓶颈。
2. **C 组放开限幅**：`|v|` 到 0.300，运动时长 20.6 s → **15.4 s（−25%）**。
   但只放开 controller、不放 `velocity_smoother` 时（B 组）只快了 13.6% —— **改参数要沿整条链路检查**。
3. **原地转占比降不下来**（21%）：说明「停下转向」不是速度上限问题，而是 **DWB 的策略选择**
   （`PathAlignCritic` 权重较高，倾向先对准再前进）。要改得调 critic 权重，不是调速度上限。

#### 实验三：waypoint 多点巡航（任务级编排）

用 `FollowWaypoints`（`nav2_msgs/action`）一次下发 5 个航点，绕场一周回起点：

```bash
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints "{
  poses: [
    {header: {frame_id: map}, pose: {position: {x:  1.5, y: -1.2, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x:  1.5, y:  1.2, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: -1.0, y:  1.2, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: -1.0, y: -1.2, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x:  0.0, y:  0.0, z: 0.0}, orientation: {w: 1.0}}}
  ]
}" --feedback
```

实测结果：

| 指标 | 值 |
|---|---|
| 航点 | 5 个全部到达（`missed_waypoints: []`） |
| 规划直线总长 | 10.78 m |
| **实际行驶轨迹** | **11.07 m（+2.7%）** |
| 运动时长 | 84.1 s |
| `\|v\|` 平均 / 最大 | 0.159 / **0.300** |
| `\|w\|` 最大 | **1.500**（= 参数上限，也用满了） |
| 原地转占比 | **33.7%**（5 次到点对准方向） |
| 全局路径条数 | 81（持续重规划） |

**可解读的三点**：

- 实际轨迹 11.07 m vs 规划 10.78 m，只多 2.7% —— 路径追踪精度不错。
- 平均有效速度 = 11.07 ÷ 84.1 = **0.132 m/s**，明显低于 `|v|` 平均 0.159 ——
  **差的就是原地转的时间**（占 33.7%）。
- 所以**多点巡航的效率瓶颈不在直线速度，而在每个航点的转向对准**。

#### 度量工具的两个坑（已修正）

1. **「时长」要按运动时间算，不是录制窗口**：`/cmd_vel` 采样数 ÷ 20 Hz 才是真正的运动时长；
   首末消息差包含了「开录过早 / 到点后未及时停录」的空闲时间。
2. **累计型指标必须归一化**：相邻速度差之和要除以采样数，否则采样多的 bag 天然显得更「抖」
   —— 未归一化时 Run A 的 1.12 反而比 Run B 的 0.40「差」，归一化后（0.0027 vs 0.0011）才可比。

对应改动已落在 `tools/bag_metrics.py`：新增 `时长`、`真停占比`、`原地转占比`、`抖动/样本`。

> **涉及文件**：`tools/bag_metrics.py`、`src/my_robot_description/config/nav2_params.yaml`

### 22. DWB critic 调优：从「原地转 21%」到「0%」

#### 问题

昨天发现多点巡航时**原地转占了 33.7%** 的时间、单点导航占 21%。
把 `max_vel_theta` 提到 1.5 也没降下来 —— 说明不是速度上限的问题。

#### 两次失败的假设

| 假设 | 改动 | 结果 |
|---|---|---|
| 朝向权重太高，DWB 想「先对准再走」 | `PathAlign.scale 32→8`、`GoalAlign.scale 24→6` | 原地转 **21.0% → 20.5%**（几乎不变），且路径从 1.74 m 变 1.82 m、时长 15.4 → 16.9 s（**反而变差**） |

**失败原因**：`PathAlign` / `GoalAlign` 控制的是**中途**的朝向贴合，而问题根本不在中途。

#### 关键一步：画时间线

把 `/cmd_vel` 按 0.5 秒分桶，标记每格状态（`.` 真停 / `R` 原地转 / `=` 前进）：

```
ctrl_v030_fix:   =======================RRRRRR...
ctrl_align8:     =========================RRRRRR...
                 ↑ 全程前进               ↑ 最后 3 秒原地转
```

**结论一目了然：原地转 100% 发生在到达目标之后**，中途一次都没有。

#### 真正的元凶：`RotateToGoal` + `yaw_goal_tolerance`

车到达目标**位置**时的朝向是 -38.7°，而目标要求 yaw=0，
`RotateToGoal` critic 会驱动它转 0.676 rad 对准；`xy_goal_tolerance: 0.25` 内还要反复微调，
实际花了 **3.0 秒**（理论只需 0.78 s）。

#### 实验数据（目标点 (1.5, -1.2)，直线 1.92 m）

| 组 | 关键改动 | `\|v\|` 平均 | 运动时长 | 真停 | **原地转** | 轨迹 |
|---|---|---|---|---|---|---|
| C | 原配置（critic 32/24，`yaw_tol 0.25`） | 0.193 | 15.4 s | 8.4% | **21.0%** | 1.74 m |
| D | `PathAlign 8` / `GoalAlign 6` | 0.202 | 16.9 s | 7.1% | 20.5% | 1.82 m |
| **E** | **`RotateToGoal 0.0` + `yaw_tol 3.15`** | **0.251** | **10.8 s** | 11.1% | **0.0%** | 1.78 m |

**E 组相对 C 组：原地转 21% → 0%，运动时长 15.4 s → 10.8 s（−30%），`|v|` 平均 +30%。**

#### 结论：没有「更好的参数」，只有「匹配需求的参数」

| 场景 | 配置 |
|---|---|
| 不在意最终朝向（送物、巡逻、扫地） | `RotateToGoal.scale: 0.0` + `yaw_goal_tolerance: 3.15` |
| 在意朝向（充电桩对接、对位拍照、机械臂作业） | 保留默认 `32.0` / `0.25`，接受那 3 秒 |

#### 方法论：先定位「哪一段」，再决定改哪个参数

这次连续两轮预测错误，根因都是**直接跳到"改哪个参数"，而没先确认"现象发生在什么阶段"**。
`tools/cmd_timeline.py` 就是为此写的 —— 三行输出就能把问题从"调参"变成"定位"。

> **涉及文件**：`tools/cmd_timeline.py`、`src/my_robot_description/config/nav2_params.yaml`

### 23. rclpy 编程与任务脚本

工作区里第一个 **`ament_python`** 包（前三个都是 `ament_cmake`）。

#### C++ ↔ Python 对照

| C++（rclcpp） | Python（rclpy） |
|---|---|
| `class Foo : public rclcpp::Node` | `class Foo(Node)`；`super().__init__('名字')` |
| `create_subscription<T>(topic, 10, cb)` | `self.create_subscription(T, topic, cb, 10)` |
| `create_wall_timer(50ms, cb)` | `self.create_timer(0.05, cb)` —— **单位是秒** |
| `declare_parameter<T>("x", 1)` + `.as_int()` | `self.declare_parameter("x", 1).value` |
| `add_executable(名 src/x.cpp)` | **`setup.py` 里的 `entry_points`** |
| `ament_target_dependencies(...)` | `package.xml` 里的 `<depend>` |

`entry_points` 的写法：`'可执行名 = 包名.模块名:main'`。

#### `pose_monitor.py`：订阅 + 定时器 + 参数

订阅 `/odom`，每秒汇报位置与累计里程（里程用相邻位姿的欧氏距离累加）。
覆盖了 rclpy 的订阅、定时器、参数、日志四种基本用法，对应 C++ 侧的
`image_processor`（订阅 + 处理）与 `cmd_vel_watchdog`（订阅 + 定时器）的组合。

#### `cmd_vel_repeater.py`：一次有价值的「错误尝试」

**动机**：曾误判"teleop 是一次性发布、被 0.5 s 看门狗超时打断"，于是写了这个
"以 10 Hz 重复最后一条指令"的节点。

**实测结论**：`teleop_twist_keyboard` 本身就是**以固定频率持续重发当前指令**的
（锁存语义），repeater 完全多余 —— 而且它会让看门狗的"断流停车"永远触发不了
（心跳被持续刷新）。

**但它作为练习是完整的**（订阅 + 发布 + 定时器 + 参数），并且对**非周期指令源**
（语音命令、REST 接口、单帧视觉指令）依然是正确的适配器模式。当前在链路上**不启用**。

#### `patrol.py`：`nav2_simple_commander` 任务编排

```python
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

nav = BasicNavigator()
nav.waitUntilNav2Active()            # 等 bt_navigator + amcl
nav.followWaypoints(poses)           # 非阻塞，返回"是否被接受"
while not nav.isTaskComplete():      # 轮询
    fb = nav.getFeedback()           # 有 current_waypoint
    ...
result = nav.getResult()             # TaskResult.SUCCEEDED / FAILED / CANCELED
```

**和 `ros2 action send_goal` 的本质区别是异步**：`followWaypoints` 立刻返回，
等待期间可以插入"拍照 / 检测 / 上报"等业务逻辑 —— 这才是任务编排。
同样的逻辑用 C++ 写约 150 行（要自己管 action client 的三套回调），Python 约 60 行。

> 两个实现细节：目标点 `header.stamp` 留 0（Nav2 视为"用最新 TF"，避免节点时钟与
> 仿真时钟不一致导致的外推错误）；`getattr(fb, 'current_waypoint', -1)` 容错不同版本字段差异。

> **涉及文件**：`src/my_robot_tasks/`（`pose_monitor.py`、`cmd_vel_repeater.py`、`patrol.py`、`setup.py`）

### 24. 动态避障实验（地图里没有的障碍）

前面的避障都是「绕开地图里已有的墙 / 方块 / 圆柱」。这次验证的是
**静态地图（`.pgm`）里不存在、只有激光能看到的障碍**。

#### 实验设计

| 项 | 做法 |
|---|---|
| 障碍 | `spawn_entity.py` 生成 0.4×0.4×0.4 m 的静态红色方块（`obstacle.sdf`）到 (1.0, 0) |
| 为什么不用 GUI | Gazebo Classic 的 `Insert → Box` **尺寸写死**，`S` 缩放工具只改视觉不改碰撞体；必须用 SDF 指定 `<size>` |
| 目标 | (2.8, 0.0) —— 直线必须经过箱子 |
| 录制 | `/cmd_vel` `/amcl_pose` `/plan` `/local_costmap/costmap` |

**放障碍的前提**：车已在原地（`/odom` x≈y≈0），且**不要按 `Reset World`** —— 那会把运行时
spawn 的模型清掉。

#### 结果：避障是「规划层」完成的，不是局部闪避

分析每条全局路径在箱子区域（x∈0.7~1.3）的最大横向偏移：

| 第几条 | 时刻 | 路径长度 | 点数 | 该区域 max\|y\| |
|---|---|---|---|---|
| 第 1 条 | 9.6 s | 3.97 m | 158 | **0.82 m** |
| 第 2 条 | 10.6 s | 3.94 m | 157 | 0.79 m |
| 第 3 条 | 11.7 s | 2.54 m | 101 | 0.21 m（车已绕过） |
| 第 16 条 | 24.5 s | 0.38 m | 14 | 0.00 m |

**第一条全局路径就已经绕开了** —— 因为 `global_costmap` 里除了 `static_layer`，还有
**`obstacle_layer`（订阅 `/scan`）**。所以：

> **「地图文件里没有」不等于「规划器不知道」** —— 激光扫到就会被标记进代价地图，
> 然后**重规划**绕开，而不是靠局部临时闪避。

#### 轨迹与代价量化

实际轨迹：`(0,0) → (0.28,0.11) → (1.23,0.87) → (2.29,0.77) → (2.79,0.33)`

```
箱子中心 (1.0, 0)，尺寸 0.4  → 占 y ∈ [-0.2, +0.2]
膨胀半径 0.55                → 高代价区边界 y = 0.2 + 0.55 = 0.75
车实际走过                    → y = 0.87，正好在边界外侧 0.12 m
```

| 指标 | 值 |
|---|---|
| 直线距离 | 2.80 m |
| **实际全局路径** | **3.97 m（+42%）** |
| 实际行驶轨迹 | 3.37 m |
| 轨迹相对首条路径的最大横向偏离 | 0.23 m（很贴路径） |
| 原地转占比 | 0.0%（`RotateToGoal: 0` 后不再到点转向） |

**绕一个 0.4 m 的箱子多走 1.17 m（42%）** —— 与 §20 中 `inflation_radius` 实验的结论一致：
膨胀半径是绕行代价的直接来源。

#### 两个待改进的发现

| 现象 | 说明 |
|---|---|
| ~~避障时没有减速~~ **（已在 §26 修正）** | 那次是**侧向通过**（箱子在 90° 方向、横向 0.87 m），本来就不该降速。而且 DWB 其实有**内在的代价感知降速**：靠近障碍时高速轨迹被否决，只能选低速轨迹（见 §26 实测 0.30 → 0.016 m/s） |
| **到点误差 0.33 m > 容差 0.25 m** | 终点 (2.79, 0.33)，目标 (2.80, 0)。若 action 显示 `SUCCEEDED`，说明 goal checker 的判定是"曾经进入过容差" |

#### 三层避障的真实分工

| 层 | 组件 | 数据来源 | 覆盖 | 实际观察到的作用 |
|---|---|---|---|---|
| 规划层 | `global_costmap` | `static_layer`(/map) + **`obstacle_layer`(/scan)** + inflation | 整图 | **本次绕行由它完成**（第一条路径已绕开） |
| 控制层 | `local_costmap` | `voxel_layer`(/scan) | 3×3 m 滚动窗口 | 实时标记，供 DWB 否决碰撞轨迹 |
| 轨迹评分 | DWB `BaseObstacle` | local costmap 代价 | 每条候选轨迹 | 否决压到障碍上的轨迹 |

> **涉及文件**：`obstacle.sdf`

### 25. IMU 接入与 EKF 传感器融合

#### 传感器本身

URDF 里加 `imu_link`（0.03³ 方块、0.02 kg）+ `<sensor type="imu">` + `libgazebo_ros_imu_sensor.so`，
`update_rate 100`，陀螺 σ=0.0002 rad/s、加速度计 σ=0.017 m/s²，话题 `/imu`。

静止实测：`orientation.w ≈ 1`、`angular_velocity ≈ 0`、**`linear_acceleration.z ≈ 9.83`**。

> **加速度计测的是「比力」而不是「运动加速度」** —— 静止时地面支撑力向上 9.81 m/s²，
> 所以 z 分量是 **+9.81**。这个值接近 9.81 恰恰说明 IMU 正常；等于 0 反而是错的。

#### EKF 配置的三个关键选择

| 选择 | 原因 |
|---|---|
| **`two_d_mode: true`** | 差速车只有 x、y、yaw 三个自由度；还避免 IMU 微小俯仰噪声污染位姿 |
| **`publish_tf: false`** | `odom → base_footprint` 已由差速插件发布，EKF 再发一次会 TF 打架 |
| **不用 IMU 的 `orientation`** | 实测 `orientation_covariance` 全是 **0**（等于宣称"绝对精确"），会让滤波器无视其他所有输入；真实 IMU 的绝对朝向也易受干扰 |
| **不用 odom 的绝对 `yaw`** | 每 20 ms 一次的绝对朝向测量会压制角速度积分，让 IMU 的速率完全失效 |

协方差对比（决定融合权重）：

| 来源 | 角速度方差 | σ | 相对权重 |
|---|---|---|---|
| `/odom` 的 `twist.covariance` | 1e-3 | 0.032 rad/s | 1 |
| `/imu` 的 `angular_velocity_covariance` | 4e-8 | 0.0002 rad/s | **25000** |

#### 实验：原地转 13 秒，比较三种「转角」（干净数据）

| 数据源 | 总转角 | 相对真实误差 |
|---|---|---|
| **IMU 角速度积分（真实）** | **310.3°** | — |
| `/odom` 的 yaw | 307.6° | −2.7°（−0.9%） |
| `/odometry/filtered` | 306.9° | −3.3°（−1.1%） |
| 融合 − odom | −0.6° | 几乎无差别 |

#### 基线实验（不注入误差）：融合看不出收益

仿真中没有打滑 —— `/odom` 的角速度与 IMU **完全一致**（都是 0.4583 rad/s），
`/odom` 的 yaw 本身误差只有 0.9%。IMU 没有任何「可纠正的误差」可做，融合结果自然等于 odom。

**「测不出收益」本身是有效结论**：在理想仿真里讨论融合优化没有意义。
要真正验证融合价值，必须**人为注入误差源**：

| 方案 | 做法 | 预期 |
|---|---|---|
| 降低车轮摩擦 | 车轮加 `<mu1>0.05</mu1><mu2>0.05</mu2>` | 打滑 → odom 过报 → 融合纠正 |
| **伪造 odom 噪声** | rclpy 节点给 `/odom` 的 yaw 加 5% 漂移，另发 `/odom_noisy` | 误差量可控、可重复 |
| 给陀螺加零偏 | SDF 噪声 `<mean>` 设 0.02 rad/s | 看 odom 能否把漂移的 IMU 拉回来 |

#### 注入误差源：融合价值得到证明

基线实验说明「odom 准的时候融合看不出收益」，那就**人为把 odom 弄坏**，看融合能不能救回来。

**做法**：写 `odom_noise` 节点（rclpy），订阅 `/odom`，把角速度乘 1.10，
并用**污染后的角速度重新积分 yaw**（保证 `twist` 与 `pose` 自洽），发到 `/odom_noisy`；
再把 EKF 的 `odom0` 换成 `/odom_noisy`（`ekf_noisy.yaml`，其余配置完全不变）。

```
Gazebo /odom（准） ─┬──────────────────────────────────────┐
                    └─▶ [odom_noise ×1.10] ─▶ /odom_noisy ─▶[EKF]─▶ /odometry/filtered
Gazebo /imu（准）  ─────────────────────────────────────────▶[EKF]
```

**结果**（原地转，积分用 `header.stamp` 的仿真时间）：

| 数据源 | 总转角 | 相对真实误差 |
|---|---|---|
| **IMU 积分（真实）** | **224.9°** | — |
| `/odom`（未污染，作参照） | 224.9° | +0.0°（0.0%） |
| **`/odom_noisy`（注入 10%）** | **247.5°** | **+22.6°（+10.0%）** |
| **`/odometry/filtered`** | **222.9°** | **−2.0°（−0.9%）** |

**偏差消除 91%，精度提升 11 倍。** EKF 只看到「打了 10% 折扣的假轮速」和「真实的 IMU」，
却输出了误差 0.9% 的角度。时间线上误差**不随时间累积** ——
轮式里程计的偏差会一路累加，IMU 的角速度不会，这正是融合的价值所在。

**残余 0.9% 的解读**：按协方差之比（25000 : 1），融合结果本应几乎完全等于 IMU。
实测说明 EKF 实际仍给了 odom 约 9% 的权重 —— 这是**过程噪声矩阵与两个传感器采样率
（100 Hz vs 50 Hz）耦合**的结果。「协方差之比」只是权重的量级估计，不是精确公式。

> **涉及文件**：`src/my_robot_tasks/my_robot_tasks/odom_noise.py`、
> `src/my_robot_description/config/ekf_noisy.yaml`

#### 排查实录：八个把结论带偏的坑

这一节花了十几轮才定位，**过程比结论更有价值**。

| # | 坑 | 现象 | 正确做法 |
|---|---|---|---|
| 1 | **改 `src/` 不等于改 `install/`** | 改了 `ekf.yaml` 但行为不变 | `ros2 pkg prefix` 取到的是 install 副本；改完 `colcon build`，或直接用 src 路径启动 |
| 2 | **参数只在节点启动时读一次** | 编译后仍无效 | 必须重启节点，并用 `ros2 param get` 验证运行时的实际值 |
| 3 | **重复启动产生同名节点** | 两轮分析结论互相矛盾 | `ros2 node list` 出现两行同名节点、`/odometry/filtered` 的 `Publisher count: 2` → 数据被两个实例混合 |
| 4 | **丢帧让积分虚高** | `/odom` 的 `∫ωdt` 与自身 `yaw` 差 12° | 积分前先验 Hz（应 50/100/10）；`trapz` 遇丢帧会按匀速补，必须跳过间隔过大的段 |
| 5 | **不做自洽性检查** | 直接采信了不可能的数据 | 同一消息内 `∫twist.ω dt` 应等于 `Δpose.yaw`；对不上说明数据或算法有问题 |
| 6 | **把「指令 vs 实际」当成「轮式 vs 真值」** | 误判「有 8% 打滑」 | 指令 0.5 → 实际 0.4583 是**指令跟踪误差**；判断里程计漂移要比 `/odom` 与 IMU，而两者一致 |
| 7 | **时间轴运算用了 bag 记录时间** | `∫ωdt` 与 `Δyaw` 差 12°、5.4°，反复怀疑数据 | 必须用 **`header.stamp`（仿真时间）**。记录时间受主机负载影响、与仿真时间不同步；改用仿真时间后自洽性立刻恢复到 ±0.2° |
| 8 | **文件名与 `entry_points` 不一致** | `ros2 run` 报 `ModuleNotFoundError: my_robot_tasks.odom_noise` | 文件存成了 `dom_noise.py`（少一个 o）。`entry_points` 冒号右边是 `包名.文件名:函数名`，必须与磁盘文件名**完全一致** |
| **`ros2 run` 起来了但什么也不干** | `collision_monitor` 无订阅、无发布，像"没工作" | 它是 **Lifecycle 节点**，日志里写着 `Waiting on external lifecycle transitions to activate`。必须 `ros2 lifecycle set ... configure/activate`，或交给 `lifecycle_manager` |
| **矩形多边形的角落比正前方更早触发** | 前向 ±30° 最近距离 0.417 m，但 0.28 m 的急停区已经触发 | 矩形四角 `(0.28, ±0.22)` 距车心 0.356 m，斜前方 40~45° 的点先落进去。统计"最近距离"时要考虑角度 |
| **录 bag 前没确认目录创建** | 做完整套实验，最后发现没有任何新 bag | `ros2 bag record` 后必须看到 `Recording...`，并另开终端 `ls -ld bags/xxx` 确认目录存在、结束后 `du -sh` 确认不是 0 |
| **`status: 4` 是成功，不是放弃** | 把 `4` 当 ABORTED，白排查一整轮 | `STATUS_SUCCEEDED = 4`、`STATUS_CANCELED = 5`、`STATUS_ABORTED = 6`。**先查枚举再解读现象** |
| **`isTaskComplete()` 会在回调里二次 spin** | 定时器里调它，结果检查永远不触发 | 它内部调 `spin_until_future_complete`。在回调/定时器里改用 `self.result_future.done()` |
| **视觉目标必须落在视野锥内且无遮挡** | 检测器一条不发，任务节点干等 | 相机水平 FOV 60° → 只有 ±30°。放目标前用 `/scan` 确认正前方最近距离就是它 |

> **可信判据 = 数据体检三条**：三个话题 Hz 符合预期（50 / 100 / 10）、
> `Publisher count` 为 1、自洽性误差在几度以内。三条都满足，结论才可信。

> **涉及文件**：`src/my_robot_description/urdf/robot.urdf`、`src/my_robot_description/config/ekf.yaml`

### 26. 第四层安全：`nav2_collision_monitor` 独立急停与降速

§24 曾把「绕箱时 0.263 m/s 满速通过」当成安全缺口。接入 `collision_monitor` 后才发现：
**那次是侧向通过（箱子在 90° 方向），本来就不该降速**。真正的缺口是「正前方突然出现障碍」。

#### 控制链：四层

```
DWB ─▶ /cmd_vel ─▶ watchdog ─▶ /cmd_vel_safe ─▶ collision_monitor ─▶ /cmd_vel_final ─▶ 差速插件
       ↑指令源         ↑数据流心跳                ↑物理距离
```

| 层 | 检查什么 | 保护对象 |
|---|---|---|
| `cmd_vel_watchdog` | 指令源还活着吗（0.5 s 无指令就停） | 遥控端崩溃 |
| **`collision_monitor`** | **车头前方有没有东西**（`/scan`） | **撞上去** |

**为什么放最末端**：它必须看到**最终**要发给电机的速度，才能保证"无论上游谁发疯，最后都过一遍急停检查"。
所以 URDF 里差速插件的 remap 从 `cmd_vel_safe` 改成了 `cmd_vel_final` —— **代价是 `collision_monitor` 不跑车就完全不动**（安全层不可绕过）。

#### 配置要点

```yaml
collision_monitor:
  ros__parameters:
    base_frame_id: base_footprint
    odom_frame_id: odom
    cmd_vel_in_topic: cmd_vel_safe
    cmd_vel_out_topic: cmd_vel_final
    source_timeout: 2.0          # 激光断流也停车
    stop_pub_timeout: 2.0
    polygons: ["PolygonStop", "PolygonSlow"]

    PolygonStop:                 # 前方 0.28 m 内有障碍 → 输出零
      type: polygon
      points: [0.28, 0.22, 0.28, -0.22, -0.10, -0.22, -0.10, 0.22]
      action_type: stop

    PolygonSlow:                 # 前方 0.70 m 内 → 速度 ×0.4
      type: polygon
      points: [0.70, 0.35, 0.70, -0.35, -0.20, -0.35, -0.20, 0.35]
      action_type: slowdown
      slowdown_ratio: 0.4

    observation_sources: ["scan"]
    scan: {type: scan, topic: /scan, enabled: true}
```

`points` 是**扁平数组** `[x1,y1, x2,y2, ...]`，相对 `base_footprint`，x 向前、y 向左。

> **`state_topic` 在 Humble 1.1.20 里不存在**（我一开始凭印象写了，实测 `ros2 param get` 报 `Parameter not set`）。
> 这个模块没有状态话题，**可观测性就是"看 `/cmd_vel_final` 的速度变没变"**，外加 `/polygon_stop` `/polygon_slowdown` 可在 RViz 里可视化。

#### 它是 Lifecycle 节点，必须激活

```
ros2 run nav2_collision_monitor collision_monitor ...
#   → "Waiting on external lifecycle transitions to activate"
#   → 停在 unconfigured，不订阅不发布，看起来像"没工作"
```

三种激活方式：

| 方式 | 命令 |
|---|---|
| 手动（调试用） | `ros2 lifecycle set /collision_monitor configure` 然后 `activate` |
| **交给 lifecycle_manager（采用）** | 见下 |
| 加进 Nav2 的 `lifecycle_manager_navigation.node_names` | 会与 Nav2 绑定，不开 Nav2 时失效 |

`gazebo.launch.py` 里的做法：

```python
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor',
            parameters=[collision_params, {'use_sim_time': True}],
            output='screen',
        ),
        # 延迟 10 秒：等车 spawn 出来、TF 与 /scan 就绪，否则 activate 失败
        TimerAction(period=10.0, actions=[
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_safety',
                parameters=[{'use_sim_time': True, 'autostart': True,
                             'node_names': ['collision_monitor']}],
                output='screen',
            ),
        ]),
```

#### 三组实验

**① 直线撞击（手动发 0.15 m/s，正前方 1.74 m 有方块）**

| 项 | 设计值 | 实测 | 判定 |
|---|---|---|---|
| slowdown 触发距离 | 0.70 m | 0.698 → 0.649 m | ✅ |
| stop 触发距离 | 0.28 m | 0.293 → 0.265 m | ✅ |
| `slowdown_ratio` | 0.40 | **0.40**（精确到采样） | ✅ |
| 边界抖动 | 无 | 全程只有 2 次速度突变 | ✅ |

三段时长：0.150 m/s 持续 7.7 s → 0.060 m/s 持续 7.2 s → 0.000 m/s。

**② 导航绕行（箱子在侧方）**

干预 **0 次**，`ratio` 全程 1.00。因为绕过箱子时它在正侧方（y≈0.87 m），
乘以 `PolygonStop` 的 y 范围 ±0.22 之外 —— **这是正确行为，不是失效**。

**③ 动态障碍（导航途中在前方 0.7 m 扔箱子）**

```
t=0.9 s  箱子出现（前向距离 1.73 → 0.66 m）
         同一采样点内  cmd 0.300 → final 0.120    ← ×0.40，<50 ms 反应
```

| 指标 | 无箱子 | 有箱子 | 变化 |
|---|---|---|---|
| 降速占比 | 0% | **35.9%** | — |
| 急停占比 | 0% | **4.4%（≈1.5 s）** | — |
| 前向最小距离 | 0.876 m | **0.352 m** | — |
| 轨迹长度 | 3.38 m | 4.36 m | +29% |
| **导航总时长** | 30.1 s | **59.6 s** | **+98%** |

**代价量化：多花 30 秒（+98%），换"没撞上突然出现的箱子"。**

#### 修正：DWB 其实会减速

§24 写过「DWB 只负责绕开，不负责靠近障碍时降速」—— **这是错的**。动态障碍实验里 Nav2 自己的输出：

```
t=2.5s  0.174 → t=5.0s  0.111 → t=10.0s  0.047 → t=15.0s  0.016
```

靠近障碍时，**所有高速候选轨迹都会被 BaseObstacle 否决**，DWB 只能选低速的可行轨迹 ——
这叫**内在的代价感知降速**（implicit cost-aware slowdown），不是显式的限速规则。

所以两者是互补的：

| 组件 | 视角 | 行为 |
|---|---|---|
| DWB | 规划 | 会绕、会因"高速轨迹全被否"而减速，但可能贴得很近 |
| `collision_monitor` | 安全 | 不管规划多好，太近就乘系数或归零 |

#### 一个反直觉的细节：矩形多边形的**角落先触发**

首次急停发生时，前向 ±30° 内的最近距离是 **0.417 m**，而 stop 多边形只到 0.28 m。原因：

```
多边形角落 (0.28, 0.22) → 距车心 0.356 m
车斜着接近时，激光点从 40~45° 方向先落入角落区域
    点 (0.21, 0.21)：距离仅 0.30 m，但 x=0.21 ≤ 0.28 → 已触发
```

**"正前方 0.28 m" 实际等于"任何方向进入这个矩形"，斜前方会更早触发。** 窄通道场景要留意这点。

> **涉及文件**：`src/my_robot_description/config/collision_monitor.yaml`、
> `src/my_robot_description/launch/gazebo.launch.py`、`src/my_robot_description/urdf/robot.urdf`

### 27. 视觉目标导航：相机 → 目标点 → Nav2

#### 目标

相机画面里找到一个红色箱子 → 算出它在 `map` 坐标系里的位置 → 让 Nav2 开过去。

#### 三段原理

```
① 像素 (u,v) ──K⁻¹──▶ 相机光学坐标系方向向量
② 方向向量 ──固定旋转 + TF──▶ map 坐标系方向
③ 从相机光心沿该方向射出，与地面 z=0 求交 ──▶ 目标在地图上的位置
```

**① 内参反投影**

```
d_opt = normalize( K⁻¹ · [u, v, 1]ᵀ )
```

`K` 从 `/camera/camera_info` 的 `k` 字段读。实测 `fx=554.4 fy=554.4 cx=320.5 cy=240.5`，
与「640 px / 水平 FOV 60°」的理论值 `320/tan(30°)=554.3` 完全吻合。

**② 光学坐标系 → 车体坐标系**（最容易搞错的一步）

| 光学坐标轴（ROS 图像约定） | 车体坐标轴 |
|---|---|
| +X（图像向右） | **−Y** |
| +Y（图像向下） | **−Z** |
| +Z（镜头朝向） | **+X** |

```python
R_OPTICAL_TO_LINK = np.array([[0.0, 0.0, 1.0],
                              [-1.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0]])
```

再用一次 `lookup_transform(map, camera_link)` 拿到相机在 map 里的位姿 —— 车在动、相机装在车上，
这些都由 TF 解决，**不硬编码任何安装位置**。

**③ 射线与地面求交**

```
光心 p（TF 的平移）、方向 d（上面算的单位向量）
地面 z = 0  →  t = -p.z / d.z
目标位置 = p + t·d
```

**这一段的本质是「靠先验假设测距」**：假设目标在地面上。

#### 两个必须知道的实现细节

**① 投影要用「底部边缘」，不能用整块重心**

箱子重心常高于相机（我们相机只有 0.08 m 高）。从相机射向箱子重心的射线是**朝上**的，
永远交不到 `z=0` → `d.z >= 0` → 返回 None。

```python
ys, xs = np.nonzero(mask)
v_bottom = int(ys.max())          # 图像里 y 最大 = 最贴近地面的那条边
band = ys >= v_bottom - 5         # 底边附近 6 行
u = float(xs[band].mean())
v = float(v_bottom)
```

**② TF 查询要「先按图像时间戳，失败退回最新」**

```python
for tag, tq in (('stamped', Time.from_msg(stamp) - Duration(seconds=0.02)),
                ('latest',  Time())):
    try:
        tf = self.tf_buffer.lookup_transform(parent, camera, tq, timeout=Duration(seconds=0.15))
        self.tf_mode = tag
        return tf
    except TransformException:
        continue
```

> **为什么需要回退**：Gazebo 相机的时间戳可能与 TF 缓存的时间窗错位
> （日志里见过图像时间戳比最新 TF 还新 0.7 s）。只用时间戳会大量失败，只用最新则车旋转时有
> `d·ω·Δt` 的滞后误差。**工程答案是"优先正确的，失败退回可用的，并把用了哪种暴露在日志里"
> —— 日志里的 `tf=stamped/latest` 就是干这个的，不静默降级。**

#### 实测精度

| 项 | 数值 |
|---|---|
| 红块像素 | 40840 px |
| 投影结果 | **(1.20, −0.45)** |
| 箱子真值中心 | (1.40, −0.60)，半宽 0.2 |
| 理论"近表面"点 | ≈ (1.22, −0.46) |
| **投影真实误差** | **≈ 2~3 cm** |

那 0.2 m 的偏差**不是误差，是语义差**：投影的是箱子**近前表面的底边**，不是中心。
对导航反而是好事 —— 目标点落在箱子表面，车正好停在前面。

#### `standoff`：目标必须退出代价地图的膨胀区

箱子的表面落在**致命代价**格子里，直接导航过去会被拒绝。要在「车→目标」方向上回退一段。
实测不同 `standoff` 的目标点代价（箱子膨胀半径 0.55 m）：

| standoff | 目标点 | 代价地图值 | 判定 |
|---|---|---|---|
| 0.30 m | (1.06, −0.37) | **99** | 致命 ❌ |
| 0.45 m | (0.92, −0.32) | **76** | 高 ⚠️ |
| 0.60 m | (0.77, −0.27) | 35 | 中 |
| **0.70 m** | (0.68, −0.24) | **0** | **自由 ✅ 临界点** |

**所以 `standoff` 默认取 0.80 m。** 但还要防止"退到车屁股后面去"，于是做自适应：

```python
eff = min(self.standoff, max(0.30, dist - 0.50))
```

`dist` 是车到视觉目标的距离。这样车离得近时目标点不会落在车旁边（否则 `xy_goal_tolerance 0.25`
会立即判成功、车一动不动）。

#### 视野锥：这次踩的最大坑

相机水平 FOV 60° → **只能看 ±30°**，且中间不能有遮挡。

曾经把红箱子放到 `(2.2, 0.3)`，那正好在**世界里原有方块 (2, 0) 的正后方** ——
完全被挡住，检测器一条都发不出来，`go_to_target` 干等了几分钟。

| 目标距离 | 允许的横向偏移（±30°） |
|---|---|
| 1.0 m | ±0.45 m |
| 1.5 m | ±0.75 m |
| 2.0 m | ±1.05 m |

**放目标前先确认两件事**：在 ±30° 锥内、且 `/scan` 正前方最近距离就是它（不是别的东西）。

#### 完整结果

```
红块像素           40840 px
检测到 map 坐标     (1.20, -0.45)
车起点              (0.00, 0.01)
自适应回退          eff = min(0.80, 1.28-0.50) = 0.78 m
导航目标            (0.47, -0.18)
车最终位置          (0.31, -0.04)
距目标              0.209 m  <  容差 0.25
navigate_to_pose    status = 4  →  SUCCEEDED ✅
```

#### 一个浪费了一整轮排查的陷阱：**`status: 4` 是成功**

```python
STATUS_UNKNOWN   = 0
STATUS_ACCEPTED  = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4      # ← 4 是成功！
STATUS_CANCELED  = 5
STATUS_ABORTED   = 6      # ← 6 才是放弃
```

把 `4` 读成 "ABORTED" 之后，看到的现象全被错误解释：

- 「`/cmd_vel` 45 秒一条都没有」→ 实际是**车已在容差内，判定成功，无需移动**
- 「controller 坏了」→ 实际 controller 完全正常（直接发 `FollowPath` 立刻驱动车走了 0.43 m）
- 「Behavior Tree 卡住」→ 实际 `ComputePathToPose` 三条路径全部 `status=4`（成功）

**教训：先确认枚举/状态码的含义，再解读现象。一个整数含义搞错，会浪费一整轮排查。**

> **涉及文件**：`src/my_robot_tasks/my_robot_tasks/red_target_detector.py`、
> `src/my_robot_tasks/my_robot_tasks/go_to_target.py`

## 项目结构

```text
ros2/
├── src/
│   ├── my_first_pkg/                 # ROS 2 示例功能包（39 个 C++ 源文件）
│   │   ├── src/                      # 各功能示例的 C++ 源码
│   │   │   ├── my_publisher.cpp            # 话题：发布
│   │   │   ├── my_subscriber.cpp           # 话题：订阅
│   │   │   ├── qos_publisher.cpp           # 话题：QoS 对比
│   │   │   ├── topic_components.cpp        # 话题：组件化
│   │   │   ├── add_server.cpp              # 服务：AddTwoInts 服务端
│   │   │   ├── add_client.cpp              # 服务：AddTwoInts 客户端
│   │   │   ├── device_status_publisher.cpp # 自定义消息：发布
│   │   │   ├── device_status_subscriber.cpp# 自定义消息：订阅
│   │   │   ├── get_device_status_server.cpp# 自定义服务：服务端
│   │   │   ├── get_device_status_client.cpp# 自定义服务：客户端
│   │   │   ├── action_server.cpp           # 动作：服务端
│   │   │   ├── action_client.cpp           # 动作：客户端
│   │   │   ├── lifecycle_demo.cpp          # 生命周期节点
│   │   │   ├── status_node.cpp             # 普通节点 + 参数 + 定时器
│   │   │   ├── status_component.cpp        # 组件化节点
│   │   │   ├── executor_single_demo.cpp    # 单线程执行器
│   │   │   ├── executor_multi_demo.cpp     # 多线程执行器
│   │   │   ├── executor_demo.cpp           # 执行器综合示例
│   │   │   ├── executor_callback_group_demo.cpp # 回调组：MutuallyExclusive / Reentrant
│   │   │   ├── task_isolation_demo.cpp     # 多回调组任务隔离
│   │   │   ├── worker_queue_demo.cpp       # 独立工作线程 + 任务队列
│   │   │   ├── loaned_message_demo.cpp     # Loaned message 零拷贝
│   │   │   ├── loaned_compare.cpp          # 零拷贝 vs 普通：延迟对比
│   │   │   ├── big_loaned_compare.cpp      # 1MB 大消息：零拷贝 vs 普通
│   │   │   ├── bigdata_benchmark.cpp       # 大消息：intra / dds / ipc 通路对比
│   │   │   ├── bigdata_pool_benchmark.cpp  # 大消息：内存池复用
│   │   │   ├── bigdata_pool_benchmark_loss.cpp # 大消息：内存池 + 丢弃策略
│   │   │   ├── latency_publisher.cpp       # 延迟测量：发布端
│   │   │   ├── latency_subscriber.cpp      # 延迟测量：订阅端（可切回调组）
│   │   │   ├── parameter_demo.cpp          # 参数声明与动态修改
│   │   │   ├── quaternion_demo.cpp         # 四元数与欧拉角
│   │   │   ├── tf2_broadcaster_demo.cpp    # TF2：广播变换
│   │   │   ├── tf2_listener_demo.cpp       # TF2：查询变换
│   │   │   ├── tf2_odom_broadcaster_demo.cpp    # TF2：里程计
│   │   │   ├── tf2_point_transform_demo.cpp     # TF2：点坐标转换
│   │   │   ├── laser_point_publisher.cpp        # TF2：激光点发布
│   │   │   ├── laser_point_transformer.cpp      # TF2：激光点跨坐标系变换
│   │   │   ├── cmd_vel_watchdog.cpp             # 速度指令看门狗（超时自动停车）
│   │   │   └── image_processor.cpp              # 相机 + cv_bridge + OpenCV 边缘检测
│   │   ├── launch/                   # launch 启动文件
│   │   │   ├── demo.launch.py
│   │   │   ├── multi_node.launch.py
│   │   │   └── image_processor.launch.py
│   │   ├── config/                   # YAML 参数配置
│   │   │   ├── parameter_demo.yaml
│   │   │   ├── params.yaml
│   │   │   └── image_processor.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── my_robot_interfaces/          # 自定义接口包
│   │   ├── msg/
│   │   │   ├── DeviceStatus.msg      # 设备状态
│   │   │   ├── LatencyData.msg       # 延迟测量（序号 + 时间戳 + 数据）
│   │   │   ├── BigData.msg           # 变长大消息（uint8[]）
│   │   │   ├── LoanedData.msg        # 小结数据（uint8）
│   │   │   └── BigLoanedData.msg     # 定长 1MB 大消息（uint8[1048576]）
│   │   ├── srv/
│   │   │   └── GetDeviceStatus.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── my_robot_tasks/               # Python 任务包（ament_python）
│   │   ├── my_robot_tasks/
│   │   │   ├── pose_monitor.py       # 订阅 /odom，定时汇报位置与累计里程
│   │   │   ├── cmd_vel_repeater.py   # 一次性指令 → 持续指令流（rclpy 发布练习，链路上不启用）
│   │   │   ├── patrol.py             # BasicNavigator 巡逻任务脚本
│   │   │   ├── odom_noise.py         # 给 /odom 注入角速度偏差，产出 /odom_noisy（融合实验用）
│   │   │   ├── red_target_detector.py # 红块检测 → 像素反投影到 map 坐标
│   │   │   └── go_to_target.py       # 订阅 /detected_target，回退 standoff 后导航过去
│   │   └── setup.py                  # entry_points 声明可执行脚本
│   └── my_robot_description/         # 机器人描述、仿真与可视化
│       ├── urdf/
│       │   └── robot.urdf            # 两轮差速小车（含 inertial/collision、雷达、相机）
│       ├── launch/
│       │   ├── display.launch.py     # 只启动 robot_state_publisher + RViz2（无物理）
│       │   └── gazebo.launch.py      # 启动 Gazebo + 生成模型 + 看门狗
│       ├── worlds/
│       │   └── my_world.world        # 仿真场景：地面 + 方块 / 圆柱 / 墙
│       ├── config/
│       │   ├── slam_toolbox.yaml     # SLAM 参数
│       │   ├── nav2_params.yaml      # Nav2 参数（已按 0.4×0.3 m 小车调整）
│       │   ├── nav2_params.baseline.yaml  # 调参实验的基线备份
│       │   ├── ekf.yaml              # robot_localization EKF 融合配置
│       │   ├── ekf_noisy.yaml        # 同上，但 odom0 指向 /odom_noisy（融合价值实验）
│       │   └── collision_monitor.yaml # 第四层安全：前方障碍降速 / 急停区域
│       ├── rviz/
│       │   └── view.rviz             # 预配置的 RViz 布局
│       ├── CMakeLists.txt
│       └── package.xml
│
├── bags/                             # rosbag2 录制数据（已 gitignore，不入库）
├── maps/                             # SLAM 产出的地图（.pgm + .yaml，体积小可入库）
├── tools/
│   ├── bag_metrics.py                # 读 bag 输出导航指标（路径长度 / 速度 / 障碍距离）
│   └── cmd_timeline.py               # 把 /cmd_vel 画成时间线，定位真停/原地转/前进的阶段
├── obstacle.sdf                      # 动态避障实验用的 0.4 m 障碍物
├── DEFAULT_FASTDDS_PROFILES.xml      # Fast DDS 自定义 profile（data_sharing 配置）
├── .gitignore
└── README.md
```

## 编译与运行

### 1. 配置 ROS 2 环境

```bash
source /opt/ros/humble/setup.bash
```

### 2. 进入工作区并编译

```bash
cd ~/ros2
colcon build
```

只编译指定功能包：

```bash
colcon build --packages-select my_robot_interfaces my_first_pkg my_robot_description
```

### 3. 加载工作区环境

```bash
source install/setup.bash
```

### 4. 运行示例

**话题通信**（分别在两个终端中运行）

```bash
# 终端一
source ~/ros2/install/setup.bash
ros2 run my_first_pkg my_subscriber

# 终端二
source ~/ros2/install/setup.bash
ros2 run my_first_pkg my_publisher
```

**服务通信**

```bash
# 终端一
ros2 run my_first_pkg add_server

# 终端二
ros2 run my_first_pkg add_client 3 5
```

**回调组与并发**

```bash
ros2 run my_first_pkg executor_callback_group_demo   # 同一回调组内的串行 / 并发
ros2 run my_first_pkg task_isolation_demo            # 多回调组任务隔离
ros2 run my_first_pkg worker_queue_demo              # 独立工作线程承接耗时操作
```

**零拷贝（Loaned Message）**

```bash
# Fast DDS 会自动加载「当前工作目录」下的 DEFAULT_FASTDDS_PROFILES.xml
cd ~/ros2
ros2 run my_first_pkg loaned_message_demo            # 先确认 can_borrow 是否支持

# 普通 vs loaned 延迟对比（参数：normal|loaned [频率Hz]）
ros2 run my_first_pkg loaned_compare normal 30
ros2 run my_first_pkg loaned_compare loaned 30

# 1MB 大消息对比
ros2 run my_first_pkg big_loaned_compare normal 10
ros2 run my_first_pkg big_loaned_compare loaned 10
```

不在 `~/ros2` 目录下运行时，用环境变量指定 profile 的绝对路径：

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/wxj/ros2/DEFAULT_FASTDDS_PROFILES.xml
```

**大消息通路对比与内存池**

```bash
# 参数：[size_MB] [频率Hz]
ros2 run my_first_pkg bigdata_benchmark intra 1 10   # 进程内
ros2 run my_first_pkg bigdata_benchmark dds   1 10   # 走 DDS
ros2 run my_first_pkg bigdata_benchmark ipc_pub 1 10 # 跨进程：发布端
ros2 run my_first_pkg bigdata_benchmark ipc_sub      # 跨进程：订阅端

ros2 run my_first_pkg bigdata_pool_benchmark          # 内存池复用
ros2 run my_first_pkg bigdata_pool_benchmark_loss     # 内存池 + 允许丢弃
```

**通信延迟测量**

```bash
# 终端一
ros2 run my_first_pkg latency_publisher

# 终端二（callback_mode 可选 mutually / reentrant）
ros2 run my_first_pkg latency_subscriber
ros2 run my_first_pkg latency_subscriber --ros-args -p callback_mode:=reentrant
```

**TF2 坐标变换**

```bash
ros2 run my_first_pkg tf2_point_transform_demo
ros2 run tf2_ros tf2_echo base_link laser      # 查看两个坐标系之间的实时变换
```

**URDF 模型与 RViz2**

```bash
ros2 launch my_robot_description display.launch.py
```

**查看当前系统中的节点和话题**

```bash
ros2 node list
ros2 topic list
ros2 topic echo /my_topic
ros2 topic hz /my_topic          # 观察实际发布频率
```

### 5. Gazebo 仿真（URDF / 雷达 / 相机 / 看门狗）

```bash
# 终端一：启动仿真（Gazebo + robot_state_publisher + 模型生成 + 看门狗）
ros2 launch my_robot_description gazebo.launch.py

# 终端二：RViz2，跟随仿真时钟
ros2 run rviz2 rviz2 \
  -d $(ros2 pkg prefix my_robot_description)/share/my_robot_description/rviz/view.rviz \
  --ros-args -p use_sim_time:=true

# 终端三：开车（注意发到 cmd_vel_input，经过看门狗）
ros2 topic pub --rate 10 /cmd_vel_input geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
ros2 topic pub --once /cmd_vel_input geometry_msgs/msg/Twist "{linear: {x: 0.0}}"

# 键盘遥控（把 teleop 的输出重映射到 cmd_vel_input）
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_input

# 终端四：图像处理（参数从 YAML 读）
ros2 launch my_first_pkg image_processor.launch.py
```

> **参数位置很关键**：`rviz2` 的 `-d` 是程序自身参数，必须放在 `--ros-args` **之前**；
> `use_sim_time` 是 ROS 参数，必须放在**之后**。写反会报 `UnknownROSArgsError`。

仿真中各话题的期望值：

| 话题 | 来源 | 期望 |
|---|---|---|
| `/clock` | Gazebo | ~1000 Hz |
| `/odom` | 差速驱动插件 | ~50 Hz |
| `/scan` | 激光雷达插件 | ~10 Hz，`sensor_msgs/LaserScan` |
| `/camera/image_raw` | 相机插件 | ~10–15 Hz，`rgb8` 640×480 |
| `/image_processed` | image_processor | ~10 Hz，`mono8` 边缘图 |
| `/cmd_vel` | cmd_vel_watchdog | 20 Hz（无指令时输出全零） |

### 6. 全部 Demo 一览

`my_first_pkg` 下共有 **35 个可执行目标**（另有 2 个组件 `status_component` / `topic_components`，
需要通过组件容器加载，不是独立可执行文件）。

下表的「运行方式」列都省略了前缀 `ros2 run my_first_pkg`，实际运行时补上即可。
**未标注参数的，直接运行就够**；标注「与 xxx 配对」的，表示需要另一个目标同时运行才能看到效果。

| 分类 | 目标 | 用途 | 运行方式 / 说明 |
|---|---|---|---|
| 话题 | `my_publisher` | 向 `/my_topic` 发布 `std_msgs/String` | 与 `my_subscriber` 配对 |
| 话题 | `my_subscriber` | 订阅 `/my_topic` 并打印 | 与 `my_publisher` 配对 |
| 话题 | `qos_publisher` | 向 `qos_topic` 发布，演示 QoS 可靠性 / 队列深度 | 用 `ros2 topic echo --qos-reliability best_effort` 等方式对照观察 |
| 服务 | `add_server` | `example_interfaces/srv/AddTwoInts` 服务端（服务名 `add_two_ints`） | 与 `add_client` 配对 |
| 服务 | `add_client` | `AddTwoInts` 客户端，两个整数由命令行传入 | `add_client 3 5` |
| 自定义消息 | `device_status_publisher` | 发布 `my_robot_interfaces/msg/DeviceStatus` | 与 `device_status_subscriber` 配对 |
| 自定义消息 | `device_status_subscriber` | 订阅 `DeviceStatus`，按温度判断 | 与 `device_status_publisher` 配对 |
| 自定义服务 | `get_device_status_server` | `my_robot_interfaces/srv/GetDeviceStatus` 服务端 | 与 `get_device_status_client` 配对 |
| 自定义服务 | `get_device_status_client` | `GetDeviceStatus` 客户端 | 与 `get_device_status_server` 配对 |
| 动作 | `action_server` | `example_interfaces/action/Fibonacci` 动作服务端 | 与 `action_client` 配对 |
| 动作 | `action_client` | Fibonacci 动作客户端，含反馈回调与取消 | 与 `action_server` 配对 |
| 生命周期 | `lifecycle_demo` | 生命周期节点 `lifecycle_demo`，五态转换 | 需配合 `ros2 lifecycle set/get`，见下方示例 |
| 节点 | `status_node` | 普通节点 + 参数 `interval_ms` + 定时器 | `--ros-args -p interval_ms:=500` |
| 执行器 | `executor_single_demo` | 单线程执行器：多个回调串行 | 直接运行 |
| 执行器 | `executor_multi_demo` | 多线程执行器：多个回调并发 | 直接运行 |
| 执行器 | `executor_demo` | 两个定时器 A / B，对比回调是否互相阻塞 | 直接运行，观察日志交错顺序 |
| 回调组 | `executor_callback_group_demo` | `MutuallyExclusive` 与 `Reentrant` 回调组的线程表现 | 直接运行，日志里打印线程 ID |
| 回调组 | `task_isolation_demo` | 多回调组把耗时任务与实时任务隔离 | 直接运行 |
| 回调组 | `worker_queue_demo` | 独立工作线程 + 任务队列承接耗时操作 | 直接运行 |
| 参数 | `parameter_demo` | 参数 `speed`：命令行 / YAML / 运行时修改 | 见上文「参数管理」一节 |
| 零拷贝 | `loaned_message_demo` | 先用 `can_borrow` 判断支持性，再跑 loaned 发布 | 需在 `~/ros2` 目录下运行（读取 Fast DDS profile） |
| 零拷贝 | `loaned_compare` | 普通发布 vs loaned 发布的延迟对比 | 第一个参数 `normal` 或 `loaned`，第二个为频率 Hz，如 `loaned_compare loaned 30` |
| 零拷贝 | `big_loaned_compare` | 同上，但消息体为 1 MB 定长 | `big_loaned_compare normal 10` / `big_loaned_compare loaned 10` |
| 大消息 | `bigdata_benchmark` | 大消息在四条通路上的对比：进程内 / DDS / 跨进程 | `bigdata_benchmark intra 1 10`、`bigdata_benchmark dds 1 10`、`bigdata_benchmark ipc_pub 1 10`（另开终端跑 `bigdata_benchmark ipc_sub`） |
| 大消息 | `bigdata_pool_benchmark` | 内存池复用缓冲区，对比分配开销 | 直接运行 |
| 大消息 | `bigdata_pool_benchmark_loss` | 内存池 + 队列满即丢弃，不阻塞发布端 | 直接运行，与上一个对照 |
| 延迟 | `latency_publisher` | 延迟测量发布端（`LatencyData` 带序号与时间戳） | 与 `latency_subscriber` 配对 |
| 延迟 | `latency_subscriber` | 延迟测量订阅端，可切换回调组 | `--ros-args -p callback_mode:=reentrant` |
| TF2 | `tf2_broadcaster_demo` | 广播 `base_link → laser` 变换 | 直接运行 |
| TF2 | `tf2_listener_demo` | 查询 `laser` / `odom` 之间的变换 | 需先有 broadcaster 在发布 |
| TF2 | `tf2_odom_broadcaster_demo` | 模拟里程计，广播 `odom → base_link` | 直接运行，位置随时间推进 |
| TF2 | `tf2_point_transform_demo` | 把一个点从 A 坐标系换算到 B 坐标系 | 直接运行 |
| TF2 | `laser_point_publisher` | 发布 `/laser_point`（`laser` 坐标系下的点） | 与 `laser_point_transformer` 配对 |
| TF2 | `laser_point_transformer` | 用 `lookupTransform` 把激光点变换到 `base_link` | 需 broadcaster 提供 `base_link → laser` |
| 仿真 | `cmd_vel_watchdog` | 速度指令看门狗：超时自动停车 | 订阅 `/cmd_vel_input`，发布 `/cmd_vel`；`--ros-args -p timeout_sec:=0.5` |
| 仿真 | `image_processor` | 相机图像 → cv_bridge → OpenCV 边缘检测 | 订阅 `/camera/image_raw`，发布 `/image_processed` |
| 其它 | `quaternion_demo` | 四元数与欧拉角互转、四元数乘法 | 直接运行，只看日志输出 |

生命周期节点的完整操作流程：

```bash
# 终端一：启动节点
ros2 run my_first_pkg lifecycle_demo

# 终端二：查看并驱动状态机
ros2 lifecycle get  /lifecycle_demo
ros2 lifecycle set  /lifecycle_demo configure    # unconfigured -> inactive
ros2 lifecycle set  /lifecycle_demo activate     # inactive     -> active
ros2 lifecycle set  /lifecycle_demo deactivate   # active       -> inactive
ros2 lifecycle set  /lifecycle_demo cleanup      # inactive     -> unconfigured
ros2 lifecycle set  /lifecycle_demo shutdown
```

URDF 模型与 RViz2 可视化通过 launch 启动（属于 `my_robot_description` 包）：

```bash
ros2 launch my_robot_description display.launch.py
```

> 提醒：`loaned_message_demo` / `loaned_compare` / `big_loaned_compare` 依赖 Fast DDS 的
> `data_sharing` 配置，需要在 `~/ros2` 目录下运行才能自动读到 `DEFAULT_FASTDDS_PROFILES.xml`。

### 7. 数据录制与离线回放

```bash
# ① 录制：显式列话题（不要 -a）
cd ~/ros2
ros2 bag record -o bags/drive_01 /scan /odom /tf /tf_static

# 录制带相机：体积大，限制时长
ros2 bag record -o bags/with_cam /camera/image_raw /scan --max-bag-duration 10

# ② 查看 bag 内容（时长 / 消息数 / 各话题类型）
ros2 bag info bags/drive_01

# ③ 回放：--clock 让播放器自己产生仿真时钟
ros2 bag play bags/drive_01 --clock 1000

# 慢放 + 循环（调参时最常用）
ros2 bag play bags/with_cam --clock 1000 -r 0.5 -l

# 只回放部分话题
ros2 bag play bags/drive_01 --clock 1000 --topics /scan /tf
```

> **验证解耦**：先 Ctrl+C 关掉 Gazebo，再执行回放 —— `/scan`、`/odom`、`/tf` 依然存在，
> 因为数据来自 bag 文件而不是仿真。此时再运行 `image_processor`，整条处理链路照常工作。

> **注意**：`-o` 后面跟的是**目录名**，不要自己加 `.db3` 后缀；
> 录制结束务必用 **Ctrl+C 正常退出**，否则 `metadata.yaml` 写不完整，
> 之后需要 `ros2 bag reindex` 修复。

### 8. SLAM 建图

```bash
# 终端一：Gazebo 仿真（建图时建议关掉 GUI，见「调试经验」）
ros2 launch my_robot_description gazebo.launch.py

# 终端二：SLAM（实时场景用 online_async）
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$HOME/ros2/src/my_robot_description/config/slam_toolbox.yaml

# 终端三：RViz，用 slam_toolbox 自带布局，Fixed Frame 设为 map
ros2 run rviz2 rviz2 \
  -d $(ros2 pkg prefix slam_toolbox)/share/slam_toolbox/config/slam_toolbox_default.rviz \
  --ros-args -p use_sim_time:=true

# 终端四：开车绕一圈（建图必须移动）
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_input

# 建好之后保存（趁 slam_toolbox 还在运行）
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

三个要点：
- RViz 的 **Fixed Frame 必须设为 `map`**，否则地图不显示
- **保存要趁 SLAM 节点还在跑** —— 地图只存在于它的内存里，Ctrl+C 之后就没了
- **建图靠「走」** —— 原地不动永远只有一帧

### 9. Nav2 自主导航

```bash
# 终端一：Gazebo
ros2 launch my_robot_description gazebo.launch.py

# 终端二：导航栈（地图与参数都必须显式传）
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/ros2/maps/my_map.yaml \
  params_file:=$(ros2 pkg prefix my_robot_description)/share/my_robot_description/config/nav2_params.yaml \
  use_sim_time:=true \
  use_composition:=False
#   等日志出现：Managed nodes are active

# 终端三：给初始位姿（否则 map→odom 不存在，代价地图会一直超时）
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: map},
  pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
}"

# 终端四：RViz（用 nav2 自带布局，Fixed Frame 已是 map）
ros2 run rviz2 rviz2 \
  -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true

# 终端五：发导航目标（导航是 action，--feedback 可看实时进度）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {header: {frame_id: map},
         pose: {position: {x: 1.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}
}" --feedback
```

`use_composition:=False` 很关键：默认 `True` 会把 10+ 个节点塞进**一个进程**，日志全混在一起，
出问题分不清是哪个节点。调试期一定要关掉。

安全层验证（这个测试最能说明架构对不对）：

```bash
# 让车跑起来，然后直接 Ctrl+C 掐掉这个发布进程 —— 车应在 0.5 秒内自动停
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
```

### 10. Python 任务脚本

```bash
# 编译 Python 包（很快，只是复制文件 + 生成入口脚本）
cd ~/ros2
colcon build --packages-select my_robot_tasks
source install/setup.bash

# ① 位置监视（只需要 /odom，Gazebo 在跑即可）
ros2 run my_robot_tasks pose_monitor

# ② 巡逻任务（需要 Gazebo + Nav2 全套）
ros2 run my_robot_tasks patrol
```

> `pose_monitor` 也可以用假数据测，不需要 Gazebo：
> ```bash
> ros2 topic pub --rate 1 /odom nav_msgs/msg/Odometry "{
>   pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}}} }"
> ```

### 11. IMU 与 EKF 融合

```bash
# ① Gazebo（提供 /clock /odom /imu）
ros2 launch my_robot_description gazebo.launch.py

# ② 确认三路输入都在
ros2 topic hz /odom      # ~50
ros2 topic hz /imu       # ~100
ros2 topic hz /clock     # ~10

# ③ 启动 EKF（注意：只启动一个！INFO 级别没有任何输出是正常的）
ros2 run robot_localization ekf_node --ros-args \
  --params-file $(ros2 pkg prefix my_robot_description)/share/my_robot_description/config/ekf.yaml

# ④ 三重验证（都过了才能开始测）
pgrep -cf ekf_node                                    # 必须 1
ros2 topic info /odometry/filtered | grep Publisher    # 必须 1
ros2 param get /ekf_filter_node odom0_config           # 第 6 个值应为 False

# ⑤ 看结果
ros2 topic hz /odometry/filtered    # ~10 Hz（受 /clock 更新率限制）
```

> **`ekf_node` 启动后一句话都不打印，这是正常的** —— 判断它是否工作只能看
> `/odometry/filtered` 话题是否存在。没输出不等于卡住，不要急着 Ctrl+C。

**融合价值实验**（把 odom 弄坏，看融合能否纠正）：

```bash
# ① 角速度 ×1.10 的假里程计：/odom -> /odom_noisy
ros2 run my_robot_tasks odom_noise

# ② EKF 改用 ekf_noisy.yaml（其中 odom0: /odom_noisy）
ros2 run robot_localization ekf_node --ros-args \
  --params-file $(ros2 pkg prefix my_robot_description)/share/my_robot_description/config/ekf_noisy.yaml
```

### 12. 四层安全链（watchdog + collision_monitor）

`collision_monitor` 已经写进 `gazebo.launch.py`，**启动 Gazebo 就会自动生效**（延迟 10 秒激活）。

```bash
# ① 启动（含第四层安全）
ros2 launch my_robot_description gazebo.launch.py

# ② 等 12~15 秒后验证
ros2 lifecycle get /collision_monitor        # active [3]
ros2 topic info /cmd_vel_safe                # Publisher 1, Subscription 1
ros2 topic info /cmd_vel_final               # Publisher 1, Subscription 1
ros2 topic hz /polygon_stop                  # ~20 Hz

# ③ 直接用 teleop 试：开向墙壁时 /cmd_vel_final 会自动降速再停
ros2 topic echo /cmd_vel_final --field linear.x
```

> ⚠️ **`collision_monitor` 不跑，车完全不能动** —— 差速插件只听 `/cmd_vel_final`，这是有意的设计，安全层不可绕过。

### 13. 视觉目标导航

```bash
# 前置：Gazebo + Nav2 已按 §12 启动，并调过 /set_initial_pose

# ① 把红箱子放到【车正前方 ±30° 锥内、无遮挡】的位置
ros2 run gazebo_ros spawn_entity.py -file obstacle.sdf -entity red_target \
  -x 1.40 -y -0.60 -z 0.2

# ② 检测器（输出像素→map 坐标）
ros2 run my_robot_tasks red_target_detector

# ③ 任务节点（收敛 5 帧 → 自适应回退 standoff → 发 Nav2 目标）
ros2 run my_robot_tasks go_to_target --ros-args -p standoff:=0.80
```

期望输出：

```
[INFO] 内参 fx=554.4 fy=554.4 cx=320.5 cy=240.5
[INFO] 红块 40840 px 底边@(553,280) -> map (1.20, -0.45)  [tf=stamped ok=1 tf失败=6 投影失败=0]
[INFO] 第 5/5 帧  (1.20, -0.45)  抖动 0.000 m
[INFO] 车到目标 1.28 m，实际回退 0.78 m（上限 0.80）
[INFO] 导航目标 (0.47, -0.18)，等 bt_navigator 上线...
[INFO] ✅ 到达目标（status=4 成功）
```

> **关键参数**：`standoff` 必须 ≥ 0.70（否则目标落在膨胀区）；
> 红箱子必须在相机 ±30° 视野锥内且无遮挡。

## 环境准备与常用命令

命令速查，避免每次重新翻找。

### 1. 依赖安装（一次性）

Ubuntu 22.04 + ROS 2 Humble。以下为本工作区实际用到的 apt 包（版本已验证）。

```bash
sudo apt update

# 基础（已含 rclcpp / rviz2 / rosbag2 / tf2 等）
sudo apt install -y ros-humble-desktop

# Gazebo 仿真
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev

# 机器人模型与状态发布
sudo apt install -y ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher

# TF 调试工具（view_frames / tf2_echo / tf2_monitor）
sudo apt install -y ros-humble-tf2-tools

# 相机图像处理
sudo apt install -y ros-humble-cv-bridge ros-humble-rqt-image-view \
  ros-humble-image-transport libopencv-dev

# 键盘遥控
sudo apt install -y ros-humble-teleop-twist-keyboard

# SLAM 建图 + 地图保存
sudo apt install -y ros-humble-slam-toolbox ros-humble-nav2-map-server

# Nav2 导航（第 7 步用，尚未配置）
sudo apt install -y ros-humble-nav2-bringup ros-humble-nav2-amcl \
  ros-humble-nav2-controller ros-humble-nav2-planner \
  ros-humble-nav2-bt-navigator ros-humble-nav2-lifecycle-manager \
  ros-humble-behaviortree-cpp-v3
```

装完建议打一个 VMware 快照 —— Nav2 依赖树深，配置出错时回滚比重装划算。

### 2. 查询速查

| 目的 | 命令 |
|---|---|
| 列出节点 / 话题 | `ros2 node list` / `ros2 topic list` |
| 话题频率 | `ros2 topic hz /scan` |
| 发布者与订阅者（含 QoS） | `ros2 topic info /scan -v` |
| 消息类型 | `ros2 topic type /scan` |
| 取一条消息的某个字段 | `ros2 topic echo /scan --once --field header.frame_id` |
| 读 latched 话题 | `ros2 topic echo /tf_static --once --qos-durability transient_local` |
| 节点的参数表 | `ros2 param list /image_processor` |
| 读 / 改参数 | `ros2 param get ... canny_low` / `ros2 param set ... canny_low 120` |
| 导出参数 | `ros2 param dump /image_processor > /tmp/p.yaml` |
| 一个节点发布/订阅了什么 | `ros2 node info /slam_toolbox` |
| bag 内容 | `ros2 bag info bags/drive_01` |
| 包安装路径（拼文件路径用） | `ros2 pkg prefix my_robot_description` |

拼接包内文件路径的常用写法：

```bash
$(ros2 pkg prefix my_robot_description)/share/my_robot_description/config/slam_toolbox.yaml
```

### 3. 仿真环境专用：TF 查询必须带 `use_sim_time`

仿真里的 TF 用**仿真时间**戳，而 `tf2_echo` / `view_frames` 默认按**系统时间**查询 ——
不加参数会报 `frame does not exist`（明明存在）。**这是最容易踩的假阴性。**

```bash
# 查两帧之间的关系
ros2 run tf2_ros tf2_echo odom base_footprint --ros-args -p use_sim_time:=true
ros2 run tf2_ros tf2_echo map odom            --ros-args -p use_sim_time:=true

# 一次性导出整棵 TF 树（比 echo 可靠，QoS 正确且容错）
ros2 run tf2_tools view_frames --ros-args -p use_sim_time:=true
evince frames_*.pdf        # 生成的 PDF
```

同理，任何带 `use_sim_time` 的节点（RViz2、slam_toolbox、AMCL）都要显式设为 `true`。

### 4. 性能与降载

Gazebo GUI（`gzclient`）单独就能吃掉近一半 CPU，调算法时它是纯浪费。

```bash
uptime                                      # load average；4 核接近 4 就是饱和
ps -eo pcpu,pmem,comm --sort=-pcpu | head -6

pkill gzclient                              # 只留 gzserver，可视化交给 RViz
pgrep -a gzserver                           # 确认服务端还活着
```

判据：`ros2 topic hz /clock` 应该在几百 Hz 以上。如果只有 10 Hz 左右，说明仿真已经
跑不动了（RTF 极低），此时话题频率、SLAM 建图都会异常 —— **先降载，再排查算法**。

## 实践与技术收获

通过这些示例，对 ROS 2 的开发流程和底层机制进行了实践，主要包括：

- 理解 ROS 2 节点、话题、服务和动作的通信模型及其适用场景。
- 掌握自定义消息与服务接口的定义、编译和跨包使用流程。
- 理解 QoS 配置对消息通信行为的影响。
- 掌握单线程 / 多线程执行器以及 Callback Group（MutuallyExclusive / Reentrant）的调度差异。
- 通过回调组划分实现任务隔离，避免耗时回调阻塞关键回调。
- 理解进程内通信、loaned message 零拷贝的前提条件与收益边界。
- 掌握 Fast DDS profile 的配置方式，理解 `data_sharing` 对零拷贝的影响。
- 掌握大消息传输中的内存分配开销，以及内存池复用、丢弃策略的取舍。
- 掌握端到端延迟的测量方法，理解回调并发方式对测量结果的影响。
- 掌握参数声明、配置文件加载和动态修改的基本方法。
- 理解 TF2 坐标树、坐标变换查询及点坐标转换流程。
- 掌握 URDF 模型的编写方式，理解 link / joint 与 TF 树的对应关系。
- 掌握 Gazebo 仿真的完整链路：`<inertial>` / `<collision>` 的作用、差速驱动插件的参数含义、
  `use_sim_time` 与仿真时钟的关系。
- 理解 `/cmd_vel` 是"设定值"而非事件，掌握用超时看门狗防止机器人失控的方法与阈值取舍。
- 理解 Gazebo 各传感器插件的 SDF 标签含义，以及话题命名（namespace / camera_name 叠加）规则。
- 掌握 `cv_bridge` 的双向转换（`toCvCopy` / `CvImage::toImageMsg`）、`rgb8` 与 `bgr8` 的差异，
  以及处理图像节点必须继承 `header` 的原因。
- 掌握 ROS 2 参数的运行时更新（`add_on_set_parameters_callback` + 校验）与持久化（YAML + launch）。
- 掌握 rosbag2 的录制与回放，理解各话题的数据量量级（`/tf` 高频、图像极大），
  以及 `--clock` 对离线回放的必要性。
- 建立"数据与算法解耦"的调试方式：一次录制、反复回放，保证算法输入的完全可复现。
- 掌握 SLAM 建图的完整链路：TF 三段的分工（`map→odom` / `odom→base_link` / 静态 TF）、
  扫描匹配与闭环检测，以及「建图必须移动」这一前提。
- 建立起一套可靠的观测方法：TF 用 `view_frames` 而不是 `echo`、仿真环境统一 `use_sim_time`、
  先看系统负载再判断「话题没数据」。
- 掌握 Nav2 导航栈的组成与依赖：地图 + AMCL 定位 + 全局规划 + 局部控制 + 行为树，
  以及「导航是 action」这一接口形态。
- 理清 `map → odom → base_link` 三段 TF 在导航中的分工，以及「AMCL 不给初始位姿
  就不发 `map → odom`，进而所有代价地图超时」这条因果链。
- 学会用「串联」而不是「抢占」组织速度指令链路：`/cmd_vel → watchdog → /cmd_vel_safe → 底盘`，
  把安全层放在所有指令的必经之路上。
- 掌握参数调优的科学方法：**固定输入、单变量、量化指标**；并理解离线回放的「开环」特性
  决定了能观测什么、不能观测什么。
- 能解读 Nav2 参数的实际影响：`inflation_radius` 从 0.55 降到 0.35，绕障路径缩短
  0.33 m（−8.8%）；而目标点若不在障碍附近，同一参数改动只带来 1.6 cm 差异。
- 理解 Nav2 速度链路的**分级限幅**：controller 想发 0.30，`velocity_smoother` 只放 0.26，
  下游不改则上游改动被静默吞掉；两处同时放开后导航时长才从 20.6 s 降到 15.4 s（−25%）。
- 能用 `FollowWaypoints` 做任务级编排：5 航点巡航 11.07 m、追踪误差仅 2.7%，
  并识别出「多点任务效率瓶颈在转向对准（33.7% 时间在原地转），而非直线速度」。
- 掌握「先定位现象发生在哪个阶段，再决定调哪个参数」的排查顺序：用 0.5 s 粒度的
  指令时间线一眼看出原地转全部集中在到点后的 3 秒，从而避免了两轮无效的参数调整。
- 理解 `RotateToGoal.scale` 与 `yaw_goal_tolerance` 的耦合：关掉朝向对准后原地转
  21% → 0%、运动时长 −30%；但两者必须成对调整，否则目标永远不判定为到达。
- 打通 Python 侧：创建 `ament_python` 包、用 `entry_points` 声明可执行脚本，
  并掌握 rclpy 的订阅 / 发布 / 定时器 / 参数四种基本用法（与 C++ 一一对应）。
- 能用 `nav2_simple_commander` 做任务级编排：`followWaypoints` 非阻塞 +
  `isTaskComplete` 轮询 + `TaskResult` 判定，等待期间可插入业务逻辑。
- 理解三层避障的真实分工：`obstacle_layer` 让「地图文件里没有的障碍」也能进入
  全局规划并被**重规划**绕开，而不是靠局部临时闪避；绕行代价（+42% 路程）直接
  由 `inflation_radius` 决定。
- 打通 IMU 接入与 EKF 融合链路：URDF 传感器、`robot_localization` 的 `two_d_mode` /
  `world_frame` / `publish_tf` 三个关键选择，以及「不用 orientation、只用 vyaw」的原因。
- 建立**数据可信性检查**的流程：话题 Hz → `Publisher count` → 自洽性（`∫ωdt` vs `Δyaw`），
  三条都过才下结论；这套检查直接避免了多次错误结论。
- 认识到「测不出收益」也是有效结论：仿真中 `/odom` 与 IMU 的角速度完全一致（0.4583 rad/s），
  融合没有可纠正的误差，要验证融合必须人为注入误差源。
- 掌握 Nav2 **LifecycleNode** 的完整用法：状态机（unconfigured → inactive → active）、
  手动 `ros2 lifecycle set`、以及用 `lifecycle_manager` + `autostart` 自动纳管。
- 能用 `nav2_collision_monitor` 搭建独立的物理安全层：多边形区域（stop / slowdown）、
  与上游 watchdog 的职责分离、以及把它作为最后一道关口插在差速插件之前。
- 理解 DWB 的**内在代价感知降速**：靠近障碍时高速候选轨迹被 BaseObstacle 否决，
  只能选低速轨迹 —— 这不是显式限速规则，而是采样评分的副产品。
- 学会用**安全代价**做量化对比：动态障碍场景下导航时长从 30.1 s 增到 59.6 s（+98%），
  换来"没有撞上突然出现的障碍"，安全与效率的权衡第一次有了具体数字。
- 打通**视觉 → 导航**的完整闭环：相机内参反投影、光学坐标系→车体的固定旋转、
  射线与地面求交、TF 位姿接入，实测投影误差仅 2~3 cm。
- 理解**单目测距的本质是先验假设**：假设目标在地面上才能把射线投到 z=0；
  因此必须投「物体底部边缘」而不是重心（重心常高于相机，射线朝上就永远交不到地面）。
- 学会把「视觉目标的物理属性」和「导航可达性」联系起来：物体表面落在代价地图的致命
  膨胀区里，必须用 `standoff` 回退；且 `standoff` 要随检测距离自适应，否则目标会落在
  车旁边导致 `xy_goal_tolerance` 立即判成功、车一动不动。
- 熟悉 ROS 2 功能包的 CMake 构建、colcon 编译和 launch 启动流程。

## 后续计划

- [x] URDF 机器人模型与机器人状态发布
- [x] RViz2 机器人模型及传感器数据可视化
- [x] Gazebo 仿真环境与带障碍物的 world
- [x] 差速驱动、里程计与键盘遥控
- [x] 速度指令看门狗（超时自动停车）
- [x] 激光雷达（`/scan`）与相机（`/camera/image_raw`）仿真
- [x] 相机图像处理：cv_bridge + OpenCV 边缘检测
- [x] 参数持久化（YAML + launch 加载）
- [x] rosbag2 录制与离线回放
- [x] SLAM 建图（slam_toolbox）
- [x] Nav2 导航框架实践（地图 + AMCL + 全局规划 + 局部控制）
- [x] Nav2 参数调优：离线实验方法 + `inflation_radius` 对比实验
- [x] 控制参数调优（controller / `velocity_smoother` 两级限幅）与 `FollowWaypoints` 多点巡航
- [x] DWB critic 调优：定位到 `RotateToGoal` 是「原地转」的真正来源
- [x] rclpy 编程（`ament_python` 包 / 订阅发布定时器 / `nav2_simple_commander` 任务脚本）
- [x] 动态避障实验（地图外障碍，验证三层避障分工）
- [x] `nav2_collision_monitor` 独立急停与降速（实测：动态障碍下启动降速 35.9%、急停 4.4%，代价为导航时长 +98%）
- [x] IMU 传感器仿真 + `robot_localization` 扩展卡尔曼融合（结论：仿真中 odom 已足够准，融合收益测不出）
- [x] 注入 odom 误差源后重做融合实验：偏差 10.0% → 融合后 0.9%（消除 91%，精度提升 11 倍）
- [ ] 单元测试（gtest / pytest）与组件容器化部署
- [ ] 平滑器（`smoother_server`）与恢复行为（`behavior_server`）实操
- [x] 相机进入导航链路：红块检测 → 像素反投影到 map → Nav2 导航（实测投影误差 2~3 cm）
- [ ] 单元测试（`pytest` 覆盖检测阈值与投影公式）
- [ ] 结合实际机器人场景完善传感器数据处理与节点协作

## 调试经验（踩坑记录）

调试仿真 + SLAM 过程中踩过的坑。共同特征是：**不报错，但结果全错**，都是一点点排出来的。

### TF 相关

| 坑 | 现象 | 原因 / 做法 |
|---|---|---|
| **`tf2_echo` 在仿真里必须加 `use_sim_time`** | 报 `frame does not exist`（明明有） | `tf2_echo` 默认按**系统时间**查，而 TF 是**仿真时间**戳。必须 `--ros-args -p use_sim_time:=true` |
| **`ros2 bag play -l` 会破坏 tf2** | `TF_OLD_DATA ignoring data from the past` 刷屏，SLAM 完全不工作 | tf2 要求时间戳单调递增，循环回放每圈都让时间倒退。**依赖 TF 的管道绝不能循环回放** |
| **bag 回放时 `/tf_static` 不可靠** | SLAM 报算不出雷达位姿，地图一直空白 | `/tf_static` 依赖 latched 语义，而 `ros2 bag play` 只当普通消息播一次。回放时应额外运行 `robot_state_publisher` 提供真正的静态 TF |
| **用 `ros2 topic echo` 判断 latched 话题空不空不可靠** | 明明有数据却读不出来 | 高负载/超时下容易假阴性。用 `ros2 run tf2_tools view_frames` 更稳 |

### 日志与观测

| 坑 | 现象 | 原因 / 做法 |
|---|---|---|
| **`output='screen'` 让节点日志不进 `launch.log`** | 事后去 `~/.ros/log/` 找不到节点输出 | 日志直接打到终端。调试期改成 `output='both'` |
| **高负载下 `ros2 topic hz` / `echo` 会假阴性** | 显示话题没发布，其实有 | 先 `uptime` 看 load —— 4 核跑 Gazebo GUI + RViz + SLAM 已饱和 |
| **Gazebo GUI 白占一半 CPU** | 仿真 RTF 只有 0.15，`/clock` 只有 10 Hz | 调算法时 `pkill gzclient`，只留 `gzserver`，可视化交给 RViz |

### 命令行

| 坑 | 现象 | 原因 / 做法 |
|---|---|---|
| **`--ros-args` 是分界线** | `UnknownROSArgsError` | 程序自身参数（如 `rviz2 -d`）必须在 `--ros-args` **之前**；ROS 参数（`-p` / `-r`）在**之后** |
| **`ros2 bag record -o` 要目录名** | 路径变成 `xxx.db3/xxx.db3_0.db3` | `-o` 是输出目录，不要自己加 `.db3` |
| **录制必须 Ctrl+C 正常退出** | `metadata.yaml` 缺失，bag 打不开 | 异常退出后用 `ros2 bag reindex` 修复 |
| **改 URDF 后必须 `colcon build`** | 改了没生效 | launch 读的是 `install/` 里的副本，不是 `src/` 原文件 |

### Nav2 相关

| 坑 | 现象 | 原因 / 做法 |
|---|---|---|
| **`map` 坐标系是 AMCL 创建的，不是 map_server** | map_server 都 active 了，仍报 `Invalid frame ID "map" ... does not exist` | map_server 只发布 `/map` **话题**；`map` 这个 **TF 坐标**要等 AMCL 收到初始位姿、发出 `map→odom` 才出现 |
| **latched 话题要设 Durability = Transient Local** | `/map` 明明在发，RViz 里空白且报 QoS warn | `map_server` 用 `TRANSIENT_LOCAL` 发布，RViz 默认 `VOLATILE` 收不到历史消息。在 RViz 里把 Map 的 Durability Policy 改成 `Transient Local` |
| **插件参数带命名空间** | `ros2 param get /controller_server max_vel_x` 报 `Parameter not set` | 它挂在 `FollowPath:` 插件下，正确读法是 `ros2 param get /controller_server FollowPath.max_vel_x` |
| **`ros2 topic info -v` 的 `Node name` 混合了发布者与订阅者** | 误以为某节点在「抢」话题 | 只看 `Publisher count` / `Subscription count`；查单个节点用 `ros2 node info <节点名>` |
| **多个发布者抢同一个 `/cmd_vel`** | 车一顿一顿、或干脆不动 | 用串联架构：`/cmd_vel → watchdog → /cmd_vel_safe → 底盘`，让底盘只监听一个话题 |
| **插件参数必须写在插件段里** | 改了 `inflation_radius` 但路径毫无变化 | `inflation_radius` / `cost_scaling_factor` 属于 `inflation_layer` 插件，必须缩进在 `inflation_layer:` 之下；写在 `ros__parameters` 直属层会被**静默忽略**。验证用 `ros2 param get <节点> inflation_layer.inflation_radius` |
| **离线回放是开环的** | 发了目标、`/cmd_vel` 全 0、车不动 | bag 里的 `/odom`、`/tf` 决定位姿，Nav2 的 `/cmd_vel` 无人消费。离线只能对比「决策」，不能对比「结果」 |
| **速度被 `velocity_smoother` 静默限幅** | `max_vel_x` 设 0.30，`/cmd_vel` 最大只有 0.26 | `velocity_smoother.max_velocity` 是链路上最后一道闸；改 controller 参数必须同步放开它，否则白改 |
| **累计型指标不归一化会得出相反结论** | 抖动 1.12 的 bag 比抖动 0.40 的更差？ | 相邻速度差之和要除以采样数；不归一化时长 bag 天然显得更差 |
| **调参前先看现象发生在「哪一段」** | 连续两轮改 critic 权重完全无效 | 把 `/cmd_vel` 按 0.5 s 分桶画时间线（`tools/cmd_timeline.py`），发现原地转 100% 在到点之后 —— 中途根本没有，改中途权重当然没用 |
| **`RotateToGoal` 与 `yaw_goal_tolerance` 必须成对改** | 只关 `RotateToGoal.scale` 后目标一直不成功 | 关掉朝向对准但朝向容差仍是 0.25 rad，车停在容差外 → goal checker 不判定到达 → BT 反复重试 |
| **遥操作是「锁存」语义** | 按一次 `j` 车就一直左转，按一次 `i` 就一直直行 | `teleop_twist_keyboard` 以固定频率持续重发当前指令，直到按 `k`。这是遥控器的正常设计（真实摇杆不回中也是保持速度），不是 bug |
| **心跳不能重复叠加** | 加了 repeater 之后，看门狗的「断流停车」失效 | repeater 持续刷新指令流，watchdog 永远等不到超时。一层心跳足够，重复叠加会让保护失效 |
| **Gazebo GUI 改不了几何尺寸** | `Insert → Box` 放下的方块没法改大小 | GUI 的 `S`（Scale）只缩视觉、不改碰撞体。必须用 SDF 文件 + `spawn_entity.py` 指定 `<size>` |
| **文档里的 `...` / `<xxx>` 是占位符** | 照抄 `map:=...` 后 launch 报 `No such file or directory: '...'` | 占位符必须替换成真实路径 |
| **重复启动会产生同名节点** | 同一份数据两轮分析结论矛盾 | `ros2 node list` 会出现两行一模一样的节点名；`ros2 topic info` 的 `Publisher count` 大于 1 → 数据被多实例混合。判断口诀：**可疑数据先查 Publisher count** |
| **积分前必须先验采样率** | `∫ωdt` 与自身 `yaw` 差 12° | 丢帧时 `trapz` 会按匀速补全间隔，结果虚高。先看话题 Hz 与中位 dt，积分时跳过间隔过大的段 |
| **静默节点的「没输出」不等于「没工作」** | `ekf_node` 启动无任何日志，误以为卡住而 Ctrl+C | 用话题判断（`ros2 topic list` / `ros2 topic hz`），而不是等日志 |

## 开发记录

| 日期 | 内容 |
|---|---|
| **2026-10-29** | 初始化工作区（`my_first_pkg` + `my_robot_interfaces`）；完成 Topic / Service / Action、自定义 msg/srv、生命周期节点、组件、执行器、参数、QoS、TF2、launch 等基础示例；建立 `.gitignore` 与初版 README |
| **2026-10-30** | 并发与性能专题：Callback Group（`MutuallyExclusive` / `Reentrant`）、任务隔离、独立工作线程；进程内通信与 loaned message 零拷贝（含 Fast DDS `data_sharing` profile）；大消息基准测试、内存池复用与丢弃策略；端到端延迟测量与 `callback_mode` 对比 |
| **2026-10-31** | ① 重写 README（项目简介 / 技术栈 / 已实现功能 / 项目结构 / 编译运行 / 技术收获 / 后续计划）；新增「全部 Demo 一览」表，并为各功能标注涉及文件<br>② 进入 Gazebo 仿真：URDF 补 `<inertial>` / `<collision>` 与万向轮；差速驱动 + 里程计；自写速度指令看门狗；激光雷达 `/scan` + 带障碍物的 world；相机 + `cv_bridge` + OpenCV 边缘检测；参数持久化（YAML + launch 加载）<br>③ rosbag2：录制 `drive_01`（4 话题 / 37 s / 3.7 MiB）与 `with_cam`（相机 / 19 s / 234 MiB）；用 `play --clock` 实现无 Gazebo 回放，验证算法与数据源解耦；`.gitignore` 排除 `bags/`<br>④ SLAM 建图：`slam_toolbox` 在线建图跑通，产出 `maps/my_map.pgm`（110×81 @ 0.05 m/px = 5.5×4.05 m）；新增「调试经验」章节，记录 TF / 日志 / 命令行三类坑<br>⑤ README 新增「环境准备与常用命令」章节：apt 依赖清单（版本已验证）、查询速查表、仿真环境 TF 查询规范、性能降载方法<br>⑥ Nav2 导航：`bringup_launch.py` 跑通完整栈，按小车尺寸调整 6 处参数，产出 `config/nav2_params.yaml`；修复 `/cmd_vel` 多发布者冲突，把看门狗改为**串联安全层**（`/cmd_vel → /cmd_vel_safe → 差速插件`）；README 补 §19 / §9 与「调试经验」Nav2 小节<br>⑦ 参数调优：建立「静态 bag + `/plan` 指标」的离线实验方法，`tools/bag_metrics.py` 支持多 bag 并排对比；实测 `inflation_radius` 0.55 → 0.35 使绕障路径 3.77 m → 3.44 m（−8.8%），并发现目标点须选在障碍附近才能测出差异<br>⑧ 在线调参与任务编排：发现 controller 与 velocity_smoother 的两级限幅（0.30 被削到 0.26），两处同时放开后导航时长 20.6 s → 15.4 s（−25%）；`FollowWaypoints` 5 航点巡航实测轨迹 11.07 m（追踪误差 2.7%），识别出瓶颈是转向对准（33.7%）；`bag_metrics.py` 增加时长 / 真停 / 原地转 / 归一化抖动 |
| **2026-11-01** | DWB critic 调优：用 0.5 s 粒度指令时间线（`tools/cmd_timeline.py`）定位出「原地转 100% 发生在到点后的朝向对准」，两轮改 `PathAlign`/`GoalAlign` 权重均无效；最终关掉 `RotateToGoal` 并放宽 `yaw_goal_tolerance` 到 3.15，原地转 21% → **0%**、运动时长 15.4 s → **10.8 s（−30%）**<br>⑨ rclpy：新建 `ament_python` 包 `my_robot_tasks`（`pose_monitor` / `cmd_vel_repeater` / `patrol`），打通 Python 侧订阅发布与 `nav2_simple_commander` 任务编排；实测确认 teleop 为「锁存」语义，repeater 在链路上多余<br>⑩ 动态避障：spawn 0.4 m 方块到 (1.0,0)，全局路径第一条即绕开（`obstacle_layer` 实时标记），绕行峰值 y=0.87、路径 3.97 m（直线 2.80 m，+42%）；发现避障时不减速（`\|v\|` 平均 0.263 m/s）<br>⑪ IMU + EKF：URDF 接入 `imu_link`（100 Hz、重力 z=9.83 验证通过），`robot_localization` 配置 `two_d_mode` + `publish_tf: false` + 只用 `vyaw`；原地转 13 秒实测 IMU 310.3°、`/odom` 307.6°（−0.9%）、`/odometry/filtered` 306.9°（−1.1%）→ **仿真中 odom 已足够准，融合收益测不出**；排查中定位到「两个 ekf_node 实例并存导致数据混合」「src≠install」「丢帧使积分虚高」等问题<br>⑫ 融合价值实验：新增 rclpy 节点 `odom_noise`（角速度 ×1.10 并用污染值重积分 yaw）+ `ekf_noisy.yaml`；注入 10.0% 偏差后融合输出误差仅 0.9%（**消除 91%，精度提升 11 倍**），误差不随时间累积；同时定位到「时间轴运算必须用 `header.stamp` 而非 bag 记录时间」，修正后自洽性从 ±12° 恢复到 ±0.2°<br>⑭ 视觉目标导航：`red_target_detector`（红块→像素反投影→map 坐标，投影误差 2~3 cm）+ `go_to_target`（多帧收敛 + 自适应 standoff + BasicNavigator）；实测车 (0,0)→(0.31,-0.04)、距目标 0.209 m < 容差 0.25、`status=4 SUCCEEDED`；修掉 `isTaskComplete()` 在回调里二次 spin、`use_sim_time` 未开、目标落入膨胀区三个问题 |<br>⑬ 第四层安全 `nav2_collision_monitor`：接入 `gazebo.launch.py` 并用 `lifecycle_manager` 延迟 10 秒自动激活；直线撞击实测触发距离 0.698/0.293 m（设计 0.70/0.28）、`slowdown_ratio` 精确 0.40、无边界抖动；动态障碍实验箱子出现后同一采样点内 0.300→0.120（<50 ms），全程降速 35.9% / 急停 4.4%，导航时长 30.1 s → 59.6 s（+98%）；同时修正了「DWB 不减速」的错误判断 |

## 说明

本仓库为个人学习与实践项目，代码主要用于理解和验证 ROS 2 相关功能。项目中的示例参考了 ROS 2 官方文档及教程，并结合个人实践进行编写和整理。
