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
│       │   └── nav2_params.baseline.yaml  # 调参实验的基线备份
│       ├── rviz/
│       │   └── view.rviz             # 预配置的 RViz 布局
│       ├── CMakeLists.txt
│       └── package.xml
│
├── bags/                             # rosbag2 录制数据（已 gitignore，不入库）
├── maps/                             # SLAM 产出的地图（.pgm + .yaml，体积小可入库）
├── tools/
│   └── bag_metrics.py                # 读 bag 输出导航指标（路径长度 / 速度 / 障碍距离）
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
- [ ] IMU 传感器仿真
- [x] rosbag2 录制与离线回放
- [x] SLAM 建图（slam_toolbox）
- [x] Nav2 导航框架实践（地图 + AMCL + 全局规划 + 局部控制）
- [x] Nav2 参数调优：离线实验方法 + `inflation_radius` 对比实验
- [ ] IMU 传感器仿真与融合
- [ ] rclpy 节点、单元测试、组件容器化部署
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

## 开发记录

| 日期 | 内容 |
|---|---|
| **2026-10-29** | 初始化工作区（`my_first_pkg` + `my_robot_interfaces`）；完成 Topic / Service / Action、自定义 msg/srv、生命周期节点、组件、执行器、参数、QoS、TF2、launch 等基础示例；建立 `.gitignore` 与初版 README |
| **2026-10-30** | 并发与性能专题：Callback Group（`MutuallyExclusive` / `Reentrant`）、任务隔离、独立工作线程；进程内通信与 loaned message 零拷贝（含 Fast DDS `data_sharing` profile）；大消息基准测试、内存池复用与丢弃策略；端到端延迟测量与 `callback_mode` 对比 |
| **2026-10-31** | ① 重写 README（项目简介 / 技术栈 / 已实现功能 / 项目结构 / 编译运行 / 技术收获 / 后续计划）；新增「全部 Demo 一览」表，并为各功能标注涉及文件<br>② 进入 Gazebo 仿真：URDF 补 `<inertial>` / `<collision>` 与万向轮；差速驱动 + 里程计；自写速度指令看门狗；激光雷达 `/scan` + 带障碍物的 world；相机 + `cv_bridge` + OpenCV 边缘检测；参数持久化（YAML + launch 加载）<br>③ rosbag2：录制 `drive_01`（4 话题 / 37 s / 3.7 MiB）与 `with_cam`（相机 / 19 s / 234 MiB）；用 `play --clock` 实现无 Gazebo 回放，验证算法与数据源解耦；`.gitignore` 排除 `bags/`<br>④ SLAM 建图：`slam_toolbox` 在线建图跑通，产出 `maps/my_map.pgm`（110×81 @ 0.05 m/px = 5.5×4.05 m）；新增「调试经验」章节，记录 TF / 日志 / 命令行三类坑<br>⑤ README 新增「环境准备与常用命令」章节：apt 依赖清单（版本已验证）、查询速查表、仿真环境 TF 查询规范、性能降载方法<br>⑥ Nav2 导航：`bringup_launch.py` 跑通完整栈，按小车尺寸调整 6 处参数，产出 `config/nav2_params.yaml`；修复 `/cmd_vel` 多发布者冲突，把看门狗改为**串联安全层**（`/cmd_vel → /cmd_vel_safe → 差速插件`）；README 补 §19 / §9 与「调试经验」Nav2 小节<br>⑦ 参数调优：建立「静态 bag + `/plan` 指标」的离线实验方法，`tools/bag_metrics.py` 支持多 bag 并排对比；实测 `inflation_radius` 0.55 → 0.35 使绕障路径 3.77 m → 3.44 m（−8.8%），并发现目标点须选在障碍附近才能测出差异 |

## 说明

本仓库为个人学习与实践项目，代码主要用于理解和验证 ROS 2 相关功能。项目中的示例参考了 ROS 2 官方文档及教程，并结合个人实践进行编写和整理。
