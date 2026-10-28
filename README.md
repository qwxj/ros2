# ROS2 学习工作区（Humble）

用 ROS2 Humble 把官方教程里的核心概念**逐个写成可运行的最小示例**，一个概念一个文件，方便对照学习。

- **2 个包**：`my_first_pkg`（26 个 demo）、`my_robot_interfaces`（自定义消息/服务）
- **约 1900 行 C++**
- 覆盖：话题 / 服务 / 动作 / 自定义接口 / 生命周期节点 / 组件 / 执行器 / 参数 / QoS / TF2 / launch

---

## 环境

| 项 | 版本 |
|---|---|
| 系统 | Ubuntu 22.04.5 LTS |
| ROS2 | Humble（`/opt/ros/humble`） |
| 编译器 | gcc / g++ 11.4.0 |
| 构建 | CMake 3.22.1 + colcon |

---

## 目录结构

```
ros2/
├── src/
│   ├── my_first_pkg/                 # 主练习包：26 个可直接运行的 demo
│   │   ├── src/                      # 各 demo 的源码（每个文件一个概念）
│   │   ├── launch/                   # launch 文件（Python 格式）
│   │   │   ├── demo.launch.py
│   │   │   └── multi_node.launch.py
│   │   ├── config/                   # 参数文件
│   │   │   ├── parameter_demo.yaml
│   │   │   └── params.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── my_robot_interfaces/          # 自定义接口包
│       ├── msg/DeviceStatus.msg
│       ├── srv/GetDeviceStatus.srv
│       ├── CMakeLists.txt
│       └── package.xml
├── build/  install/  log/            # colcon 产物（已 gitignore）
├── my_topic_bag/                     # rosbag 录制数据（已 gitignore）
└── .gitignore
```

---

## 编译与运行

```bash
# 1. 载入 ROS2 环境
source /opt/ros/humble/setup.bash

# 2. 编译全部
cd ~/ros2
colcon build

# 3. 载入本工作区
source install/setup.bash

# 4. 运行任意一个 demo（目标名见下表）
ros2 run my_first_pkg my_publisher
```

只编一个包：`colcon build --packages-select my_first_pkg`
清理重编：`rm -rf build install log && colcon build`

---

## 文件功能说明

### 一、话题通信（Topic）

| 文件 | 编译目标 | 功能 |
|---|---|---|
| `src/my_publisher.cpp` | `my_publisher` | 向话题 `my_topic` 发布 `std_msgs/msg/String`，定时器驱动 |
| `src/my_subscriber.cpp` | `my_subscriber` | 订阅 `my_topic`，收到消息打印出来 |
| `src/qos_publisher.cpp` | `qos_publisher` | 演示 **QoS**：可靠性（RELIABLE / BEST_EFFORT）、历史深度对收发的实际影响 |
| `src/topic_components.cpp` | （组件） | 把发布/订阅改写成 `rclcpp_components` 组件，可被组合进同一个进程 |

```bash
# 开两个终端，先跑订阅者再跑发布者
ros2 run my_first_pkg my_subscriber
ros2 run my_first_pkg my_publisher
```

### 二、服务通信（Service）

| 文件 | 编译目标 | 功能 |
|---|---|---|
| `src/add_server.cpp` | `add_server` | 标准服务端：`example_interfaces/srv/AddTwoInts`，服务名 `add_two_ints` |
| `src/add_client.cpp` | `add_client` | 调用 `add_two_ints`，发两个整数求和 |
| `src/get_device_status_server.cpp` | `get_device_status_server` | **自定义服务** `GetDeviceStatus` 的服务端，按设备名返回在线状态和温度 |
| `src/get_device_status_client.cpp` | `get_device_status_client` | 调用自定义服务 `GetDeviceStatus` |

```bash
ros2 run my_first_pkg add_server
ros2 run my_first_pkg add_client 3 5
```

### 三、动作通信（Action）

| 文件 | 编译目标 | 功能 |
|---|---|---|
| `src/action_server.cpp` | `action_server` | `rclcpp_action::create_server`，支持**反馈 + 中途取消**的长时间任务 |
| `src/action_client.cpp` | `action_client` | `rclcpp_action::create_client`，处理反馈回调与结果回调 |

> 服务 = 一问一答；动作 = 可取消、有进度反馈的"长任务"。

### 四、自定义接口（my_robot_interfaces）

| 文件 | 功能 |
|---|---|
| `msg/DeviceStatus.msg` | 自定义消息：`string device_name` / `bool online` / `float64 temperature` |
| `srv/GetDeviceStatus.srv` | 自定义服务：请求 `string device_name` → 响应 `bool success` / `string message` / `float64 temperature` |

| 使用它的文件 | 编译目标 | 功能 |
|---|---|---|
| `src/device_status_publisher.cpp` | `device_status_publisher` | 发布自定义 `DeviceStatus` 消息，模拟设备状态上报 |
| `src/device_status_subscriber.cpp` | `device_status_subscriber` | 订阅 `DeviceStatus`，按温度做判断/打印 |

```bash
ros2 interface show my_robot_interfaces/msg/DeviceStatus
ros2 interface show my_robot_interfaces/srv/GetDeviceStatus
```

### 五、节点生命周期与组件（Lifecycle / Component）

| 文件 | 编译目标 | 功能 |
|---|---|---|
| `src/lifecycle_demo.cpp` | `lifecycle_demo` | **生命周期节点**：`on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` / `on_shutdown` 五个状态回调 |
| `src/status_node.cpp` | `status_node` | 普通节点 + 参数 `interval_ms` + 定时器，周期性输出状态 |
| `src/status_component.cpp` | （组件） | 把 `status_node` 改写成 `rclcpp_components` 组件 |

```bash
ros2 run my_first_pkg lifecycle_demo
# 另开终端，手动驱动状态机：
ros2 lifecycle set /lifecycle_demo configure
ros2 lifecycle set /lifecycle_demo activate
ros2 lifecycle get /lifecycle_demo
```

### 六、执行器（Executor）

| 文件 | 编译目标 | 功能 |
|---|---|---|
| `src/executor_single_demo.cpp` | `executor_single_demo` | **单线程执行器** `SingleThreadedExecutor`：多个回调串行执行 |
| `src/executor_multi_demo.cpp` | `executor_multi_demo` | **多线程执行器** `MultiThreadedExecutor`：多个回调并行执行 |
| `src/executor_demo.cpp` | `executor_demo` | 两个定时器（A / B），对比单线程与多线程下"回调是否互相阻塞" |

> 这是理解 ROS2 并发模型最直接的一组例子。

### 七、参数（Parameter）

| 文件 | 编译目标 | 功能 |
|---|---|---|
| `src/parameter_demo.cpp` | `parameter_demo` | `declare_parameter<double>("speed")`、读取参数、注册参数变更回调 |
| `config/parameter_demo.yaml` | — | 参数文件（供 `--params-file` 使用） |
| `config/params.yaml` | — | 另一份参数文件，演示 YAML 结构 |

```bash
# 从命令行传参
ros2 run my_first_pkg parameter_demo --ros-args -p speed:=2.5
# 从 YAML 传参
ros2 run my_first_pkg parameter_demo --ros-args --params-file src/my_first_pkg/config/parameter_demo.yaml
# 运行中动态修改
ros2 param set /parameter_demo speed 3.0
```

### 八、TF2 坐标变换

| 文件 | 编译目标 | 功能 |
|---|---|---|
| `src/tf2_broadcaster_demo.cpp` | `tf2_broadcaster_demo` | `TransformBroadcaster::sendTransform` 广播坐标变换（父子坐标系 + 平移 + 旋转） |
| `src/tf2_listener_demo.cpp` | `tf2_listener_demo` | `Buffer` + `TransformListener` + `lookupTransform` 查询变换 |
| `src/tf2_odom_broadcaster_demo.cpp` | `tf2_odom_broadcaster_demo` | 模拟里程计：按 `speed` 参数推进机器人位置并广播 `odom → base_link` |
| `src/tf2_point_transform_demo.cpp` | `tf2_point_transform_demo` | 把一个点从坐标系 A 换算到坐标系 B（`tf2_geometry_msgs`） |
| `src/quaternion_demo.cpp` | `quaternion_demo` | 四元数与欧拉角互转、四元数乘法 —— 姿态表示的数学基础 |
| `src/laser_point_publisher.cpp` | `laser_point_publisher` | 发布激光雷达点数据（配合下面的变换练习） |
| `src/laser_point_transformer.cpp` | `laser_point_transformer` | 用 `lookupTransform` 把激光点从 `laser` 坐标系变换到 `base_link` |

```bash
# 可视化 TF 树（需要 GUI 或 rviz2）
ros2 run tf2_tools view_frames      # 生成 frames_*.pdf / *.gv
ros2 run tf2_ros tf2_echo base_link laser
```

### 九、launch

| 文件 | 功能 |
|---|---|
| `launch/demo.launch.py` | 用 Python 写 launch：启动节点、传参、重映射话题 |
| `launch/multi_node.launch.py` | 一次启动多个节点，演示 launch 的组织方式 |

```bash
ros2 launch my_first_pkg demo.launch.py
ros2 launch my_first_pkg multi_node.launch.py
```

---

## 常用调试命令速查

```bash
ros2 node list                    # 当前有哪些节点
ros2 topic list                   # 有哪些话题
ros2 topic echo /my_topic         # 看话题内容
ros2 topic hz /my_topic            # 话题频率
ros2 service list                 # 有哪些服务
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
ros2 param list                    # 参数列表
ros2 param get /node_name param    # 读参数
ros2 interface list                # 所有可用接口
ros2 run rqt_graph rqt_graph       # 图形化看节点/话题连接
```

---

## 学习进度

- [x] 话题发布 / 订阅
- [x] 服务端 / 客户端
- [x] 动作服务端 / 客户端
- [x] 自定义消息与服务接口
- [x] 生命周期节点
- [x] 组件（rclcpp_components）
- [x] 单线程 / 多线程执行器
- [x] 参数（命令行 / YAML / 动态修改）
- [x] QoS
- [x] TF2 广播 / 监听 / 点变换 / 四元数
- [x] launch 文件
- [ ] URDF / 机器人建模
- [ ] Nav2 导航
- [ ] MoveIt2 机械臂

---

## 关于 .gitignore

已排除以下内容，避免把可再生成或体积大的文件提交进版本库：

| 规则 | 原因 |
|---|---|
| `build/` `install/` `log/` | colcon 构建产物，`colcon build` 可完全重建 |
| `my_topic_bag/` `*.bag` `*.db3` `*.mcap` | rosbag 录制数据，体积大且是二进制 |
| `frames_*.gv` `frames_*.pdf` | `tf2_tools view_frames` 生成的临时图 |
| `__pycache__/` `*.pyc` | Python 字节码缓存 |

> 如果想把 TF 树的历史变化也纳入版本管理，把 `frames_*` 那两行删掉再提交即可。

---

## License

学习用途。代码参考 ROS2 官方教程（Apache-2.0）编写。
