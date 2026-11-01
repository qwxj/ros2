# rclpy 任务脚本与动态避障

> 本文是 README「已实现功能」的拆分子集，收录 **§23 ~ §24**（2 个小节）。
> 原始完整文档： [`docs/archive/README-2026-11-01-full.md`](../archive/README-2026-11-01-full.md) 第 623~752 行。

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
