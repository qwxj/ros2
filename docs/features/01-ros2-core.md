# ROS 2 核心通信与节点

> 本文是 README「已实现功能」的拆分子集，收录 **§1 ~ §9**（9 个小节）。
> 原始完整文档： [`docs/archive/README-2026-11-01-full.md`](../archive/README-2026-11-01-full.md) 第 134~232 行。

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
