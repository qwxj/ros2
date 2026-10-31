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

### 1. Topic 话题通信

- 实现 Publisher / Subscriber 发布订阅模型。
- 使用定时器周期性发布消息，订阅端接收并处理消息。
- 通过 QoS 配置示例，实践 RELIABLE、BEST_EFFORT 等可靠性策略及历史队列深度。
- 使用 `rclcpp_components` 将节点封装为组件，了解组件化节点的组织方式。

### 2. Service 服务通信与自定义接口

- 基于 `example_interfaces/srv/AddTwoInts` 实现服务端和客户端。
- 创建自定义消息 `DeviceStatus`，用于描述设备名称、在线状态和温度信息。
- 创建自定义服务 `GetDeviceStatus`，根据设备名称返回设备状态信息。
- 实践自定义接口包的定义、编译及在其他功能包中的调用。

### 3. Action 动作通信

- 使用 `rclcpp_action` 实现 Action Server 和 Action Client。
- 实践长时间任务的异步执行、过程反馈及任务取消。
- 理解 Topic、Service 与 Action 在通信场景上的区别。

### 4. 节点生命周期与组件

- 编写 Lifecycle Node，实践 configure、activate、deactivate、cleanup、shutdown 等状态转换。
- 实现普通节点及其参数、定时器功能。
- 将节点封装为 `rclcpp_components` 组件，了解组件注册与加载方式。

### 5. Executor 与回调组并发控制

- 使用 `SingleThreadedExecutor` 实践单线程回调调度。
- 使用 `MultiThreadedExecutor` 实践多线程回调调度。
- 通过多个定时器的执行示例，观察回调执行与阻塞现象，理解 ROS 2 执行器的基本工作方式。
- 使用 `MutuallyExclusive` 与 `Reentrant` 两种 Callback Group，对比同一回调组内串行执行与并发执行的区别。
- 通过多回调组 + 多节点组合，实践**任务隔离**：把耗时回调与实时性要求高的回调放进不同组，避免相互阻塞。
- 用独立工作线程承接耗时操作，把回调本身保持轻量，避免拖慢执行器。

### 6. 进程内通信与零拷贝（Loaned Message）

- 使用 `borrow_loaned_message` / `publish_loaned` 实现**零拷贝**消息发布，跑通 loaned message 完整链路。
- 通过 `can_borrow` 判断当前 RMW 是否支持 loaned message，并打印结果。
- 编写 `DEFAULT_FASTDDS_PROFILES.xml`，为 `/loaned_topic` 单独配置 `data_sharing`：
  默认 profile 使用 `OFF`，`/loaned_topic` 使用 `AUTOMATIC`，从而启用 Fast DDS 的数据共享（共享内存）通路。
- 对比**普通发布**与 **loaned 发布**的端到端延迟，量化零拷贝带来的收益。
- 实测结论：1 MB 消息、10 Hz 条件下，loaned 方式相比普通方式约有 **30%** 的延迟下降。

### 7. 大块数据传输与内存池复用

- 定义两类大消息接口：
  - `BigData`：`uint8[] payload`（**变长**，每次发布都要分配）；
  - `BigLoanedData`：`uint8[1048576] payload`（**定长 1 MB**，可用于 loaned message）。
- 对比变长消息与定长消息在不同通信方式下的表现。
- 引入**对象池 / 内存池**复用缓冲区，避免每帧重复分配释放，对比复用前后的开销。
- 提供两种发送策略用于对照：
  - `bigdata_pool_benchmark`：正常入队策略；
  - `bigdata_pool_benchmark_loss`：**允许丢弃**（队列满即丢），避免发布端被慢订阅者拖住。

### 8. 通信延迟测量与问题排查

- 编写 `LatencyData` 消息，携带序号与发送时间戳，用于计算端到端延迟。
- `latency_publisher` / `latency_subscriber` 组成最小延迟测量链路。
- `latency_subscriber` 提供 `callback_mode` 参数，可在 `mutually`（默认）与 `reentrant` 两种回调组之间切换，
  用于观察**回调并发方式对测量结果的影响**。
- `bigdata_benchmark` 支持切换通信通路做对照测试：`intra`（进程内）、`dds`（走 DDS）、`ipc_pub` / `ipc_sub`（跨进程）。

### 9. 参数管理

- 使用 `declare_parameter` 声明参数并读取参数值。
- 通过命令行和 YAML 文件传递节点参数。
- 注册参数变更回调，实践运行过程中的动态参数修改。

### 10. TF2 坐标变换

- 使用 `TransformBroadcaster` 发布坐标系之间的变换关系。
- 使用 `TransformListener`、`Buffer` 和 `lookupTransform` 查询坐标变换。
- 模拟里程计数据，广播 `odom → base_link` 坐标变换。
- 使用 `tf2_geometry_msgs` 实现不同坐标系下的点坐标转换。
- 实践四元数与欧拉角转换、四元数乘法等姿态表示基础。
- 实现激光雷达点数据发布，以及从 `laser` 坐标系到 `base_link` 坐标系的点变换。

### 11. URDF 机器人模型与 RViz2 可视化

- 新建 `my_robot_description` 功能包，用于存放机器人模型与可视化配置。
- 手写两轮差速小车的 URDF：`base_footprint` → `base_link` 固定关节，左右轮为 `continuous` 关节。
- 通过 `robot_state_publisher` 将 URDF 转换为 TF，并在 RViz2 中显示模型与坐标系。
- 使用 `joint_state_publisher_gui` 拖动滑块驱动轮子关节，观察关节角度与 TF 的联动。
- RViz 配置以文件形式随包安装（`rviz/view.rviz`），launch 时通过 `-d` 加载，无需每次手动配置显示项。

### 12. Launch 启动管理

- 使用 Python 编写 launch 文件。
- 实践多个节点的统一启动、参数传递及话题重映射。
- 通过 launch 文件组织多个节点的运行流程。

## 项目结构

```text
ros2/
├── src/
│   ├── my_first_pkg/                 # ROS 2 示例功能包（37 个 C++ 源文件）
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
│   │   │   └── laser_point_transformer.cpp      # TF2：激光点跨坐标系变换
│   │   ├── launch/                   # launch 启动文件
│   │   │   ├── demo.launch.py
│   │   │   └── multi_node.launch.py
│   │   ├── config/                   # YAML 参数配置
│   │   │   ├── parameter_demo.yaml
│   │   │   └── params.yaml
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
│   └── my_robot_description/         # 机器人描述与可视化
│       ├── urdf/
│       │   └── robot.urdf            # 两轮差速小车
│       ├── launch/
│       │   └── display.launch.py     # 启动 robot_state_publisher + RViz2
│       ├── rviz/
│       │   └── view.rviz             # 预配置的 RViz 布局
│       ├── CMakeLists.txt
│       └── package.xml
│
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
- 熟悉 ROS 2 功能包的 CMake 构建、colcon 编译和 launch 启动流程。

## 后续计划

- [x] URDF 机器人模型与机器人状态发布
- [x] RViz2 机器人模型及传感器数据可视化
- [ ] 传感器模型：激光雷达 / 相机 / IMU 的 Gazebo 仿真插件
- [ ] Gazebo 仿真环境下的差速驱动与键盘遥控
- [ ] SLAM 建图（slam_toolbox）
- [ ] Nav2 导航框架实践
- [ ] 结合实际机器人场景完善传感器数据处理与节点协作

## 说明

本仓库为个人学习与实践项目，代码主要用于理解和验证 ROS 2 相关功能。项目中的示例参考了 ROS 2 官方文档及教程，并结合个人实践进行编写和整理。
