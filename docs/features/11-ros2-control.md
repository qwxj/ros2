# ros2_control：自写硬件接口与独立导航栈

> 本文是 README「已实现功能」的拆分子集，收录 **§33**。
> 本包与 `my_robot_description`（Gazebo 方案）互为对照。

---

### 33. ros2_control 自写硬件接口：把驱动层从物理引擎里剥出来

#### 为什么要另起一个包

`my_robot_description` 里的底盘驱动是 Gazebo 的黑盒插件：

```xml
<plugin filename="libgazebo_ros_diff_drive.so">
```

它把「驱动」和「物理」焊死在一起 —— 好处是仿真真实，坏处是：
- 换实车时这段逻辑要全部重写
- `read()` / `write()` 的时机完全看不见，**没法单元测试**

`my_robot_control` 把这两件事拆开：**用 ros2_control 自己实现驱动层，用纯运动学代替物理引擎。**

| | `my_robot_description` | **`my_robot_control`** |
|---|---|---|
| 底盘驱动 | Gazebo 插件（黑盒） | **自写 `SystemInterface`（全透明）** |
| `/odom` 由谁发 | Gazebo 插件 | **`diff_drive_controller`** |
| 激光 | Gazebo 射线传感器 | **自写 ray-casting 节点** |
| 地图 | `slam_toolbox` 建图 | **Python 脚本程序化生成** |
| 定位 | AMCL + 两级 EKF | **AMCL（无 EKF）** |
| 物理 | Gazebo 引擎（碰撞/摩擦） | **无物理，纯运动学积分** |
| 时钟 | `use_sim_time: true`（`/clock`） | **`use_sim_time: false`（墙钟）** |
| 启动耗时 | ~70 s | **~3 s** |
| 可测试性 | 只能集成测试 | **`read/write` 可单元测试** |

#### 架构

```
                     ┌──────────────────────────────────────────┐
                     │  Nav2（navigation_launch.py）             │
                     │  controller_server / planner_server       │
                     │  bt_navigator / behavior_server / ...     │
                     └───────────┬──────────────┬───────────────┘
                       /cmd_vel  │              │  需要 /scan + /map + TF
                                 ▼              ▼
                  ┌──────────────────┐   ┌───────────────────┐
                  │ cmd_vel_adapter  │   │ map_server + amcl │
                  │ 私有话题桥接      │   │ 粒子滤波定位       │
                  └────────┬─────────┘   └────────┬──────────┘
   /diff_drive_controller/cmd_vel_unstamped       │ /scan
                           ▼                      ▲
              ┌────────────────────────┐   ┌──────┴─────────────┐
              │ diff_drive_controller  │   │ laser_scan_simulator│
              │ joint_state_broadcaster│   │ (ray-casting 360 线)│
              └───────────┬────────────┘   └────────────────────┘
                          ▼  read() / write()  @100 Hz
              ┌────────────────────────────────────┐
              │ DiffDriveHardware（自写插件）        │
              │  write: 模拟电机  read: 模拟编码器   │
              └────────────────────────────────────┘
```

#### 自写 `SystemInterface` 的 7 个回调

`src/diff_drive_hardware.cpp`（165 行）：

| 回调 | 作用 | 本实现 |
|---|---|---|
| `on_init` | 读 `<ros2_control>` 描述、分配成员变量 | 校验 `joints.size() == 2` |
| `export_state_interfaces` | 告诉框架「我能提供哪些状态」 | 每轮 `position` + `velocity` |
| `export_command_interfaces` | 「我能接受哪些指令」 | 每轮 `velocity` |
| `on_activate` | 上电，状态清零 | `fill(..., 0.0)` |
| `on_deactivate` | 下电 | 速度/指令清零 |
| `read` | **硬件 → 框架**（编码器） | `position_[i] += velocity_[i] * dt` |
| `write` | **框架 → 硬件**（电机） | `velocity_[i] = clamp(cmd, ±10.0)` |

**最关键的一点**：`export_*_interfaces` 返回的是**指针**，不是值：

```cpp
state_interfaces.emplace_back(joint_name, HW_IF_POSITION, &position_[i]);
//                                                       ↑ 地址
```

控制器直接通过这个指针读写你的成员变量，所以**成员变量的生命周期必须覆盖整个运行期**（这就是为什么 `position_` 是类成员 `std::vector<double>`，而不是局部变量）。

#### pluginlib + CMake export targets（两行都不能少）

```cmake
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_plugin.xml)
ament_export_targets(export_my_robot_control HAS_LIBRARY_TARGET)
```

配套的 `my_robot_hardware_plugin.xml`：

```xml
<library path="my_robot_control">
  <class name="my_robot_control/DiffDriveHardware"
         type="my_robot_control::DiffDriveHardware"
         base_class_type="hardware_interface::SystemInterface"/>
</library>
```

CPP 末尾还需要：

```cpp
PLUGINLIB_EXPORT_CLASS(my_robot_control::DiffDriveHardware, hardware_interface::SystemInterface)
```

URDF 里用 `<plugin>` 引用：

```xml
<ros2_control name="DiffDriveSystem" type="system">
  <hardware><plugin>my_robot_control/DiffDriveHardware</plugin></hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  ...
</ros2_control>
```

#### 启动时序

`launch/robot.launch.py` 依次拉起 6 个东西：

| 顺序 | 节点 | 作用 |
|---|---|---|
| 1 | `cmd_vel_adapter` | 私有话题桥接（见下方坑 ②） |
| 2 | `robot_state_publisher` | `base_link → 轮子/激光` 的静态 TF |
| 3 | `ros2_control_node`（`controller_manager`） | 加载硬件插件，100 Hz 主循环 |
| 4 | `spawner joint_state_broadcaster` | 激活关节状态发布（100 Hz） |
| 5 | `spawner diff_drive_controller` | 激活差速控制器（50 Hz 发 odom） |
| 6 | `laser_scan_simulator` | 360 线 ray-casting，10 Hz `/scan` |

**顺序有讲究**：两个 `spawner` 必须在 `controller_manager` 之后；`laser_scan_simulator` 只是话题生产者，位置随意。

#### 踩到的四个坑

**① `build/ install/ log/` 被建在了 `src/my_robot_control/` 里**

在包目录里跑了 `colcon build`。后果不只是脏 —— **两套 `install/` 混用**会导致 `source` 了哪个决定你跑的是哪份代码。

**修法**：站在工作区根编译。
```bash
cd ~/ros2 && rm -rf src/my_robot_control/{build,install,log}
colcon build --packages-select my_robot_control
```

**② `diff_drive_controller` 用私有话题 —— 车完全不动（但 TF 正常，极具迷惑性）**

```
diff_drive_controller 订阅  ~/cmd_vel_unstamped  →  /diff_drive_controller/cmd_vel_unstamped
                      发布  ~/odom               →  /diff_drive_controller/odom
```

- 每个 controller 都是**独立节点**，`~` 展开成 `/节点名/...`
- 而 `use_stamped_vel: false` 时订阅名还带 `_unstamped` 后缀（改成 `true` 则变 `cmd_vel` + `TwistStamped`）
- **Humble 的 `spawner` 不支持 remap**（`--help` 里没有这个选项）
- 最要命的是：`/odom` 不存在，但 **`odom→base_link` TF 是正常的**（由 controller 直接发），所以 `ros2 run tf2_ros tf2_echo` 看着一切正常，实际 Nav2 发出的 `/cmd_vel` 没人听

**修法**：写一个适配器节点（`my_robot_tasks/cmd_vel_adapter.py`）做双向桥接：

```python
# 下行
self.pub_cmd = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
# 上行
self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
self.create_subscription(Odometry, '/diff_drive_controller/odom', self.on_odom, 10)
```

**③ `laser_scan_simulator` 没被任何 launch 引入**

编译出来了、手动 `ros2 run` 能出 `/scan`，但 `robot.launch.py` 和 `nav2_standalone.launch.py` 里都没引用它 →
`/scan` 不存在 → AMCL 收不到激光 → **全局代价地图空、定位失效**。

**修法**：加进 `robot.launch.py`（9 行）。

**④ 静态 TF 假定位**

```python
static_transform_publisher --x 0 --y 0 ... --frame-id map --child-frame-id odom
```

等于「假设机器人永远在原点」。Nav2 能规划、能走，但**走偏之后再也纠不回来**。

**修法**：换成 AMCL（从同一份 `nav2_params_standalone.yaml` 的 `amcl:` 段读参数）：

```python
amcl = Node(package='nav2_amcl', executable='amcl', name='amcl',
            parameters=[{'use_sim_time': False}, params_file])

# lifecycle_manager 里加上 amcl
'node_names': ['map_server', 'amcl']
```

#### 实测数据

| 项 | 值 | 配置 | 达成 |
|---|---|---|---|
| `/scan` | **10.008 Hz** | 10 | ✅ 360 线，有效点 360/360，最近 0.50 m |
| `/joint_states` | **99.98 Hz** | 100 | ✅ |
| `/odom` | 49.9 Hz | 50 | ✅ |
| `map → odom` | **10.0 Hz** | AMCL | ✅（不再是静态 TF）|
| `odom → base_link` | 49.7 Hz | 50 | ✅ |
| **轮速响应** | **5.000 rad/s** | 发 0.25 m/s ÷ 0.05 m = 5.0 | ✅ |
| **odom 积分** | **1.0000 m** | 0.25 m/s × 4 s = 1.0 | ✅ |
| **自主导航** | **status=4 SUCCEEDED** | (0,0) → (2.5,−1.5) | ✅ 13.5 s，距目标 **0.244 m** < 容差 0.25 |

#### 一个沉默失败：`inflation_radius` 决定「车停在哪」

`nav2_params_standalone.yaml`：

```yaml
footprint: "[[0.15,0.1625],[0.15,-0.1625],[-0.15,-0.1625],[-0.15,0.1625]]"   # 0.30 × 0.325 m
inflation_radius: 0.30
```

**净空需求 = 0.30 + 0.1625 = 0.46 m。**

测试时车停在离障碍物 **0.34 m** 处 → **起点落在致命代价区** → 规划器找不到合法起点 →
`bt_navigator` 只报一句 `Goal failed`，**1.5 秒就 ABORTED**，日志里完全看不出原因。

**这就是 Nav2 的典型「沉默失败」**：起点不合法时，它不告诉你「你的起点在墙里」。

> **排查手法**：算一下「车到最近障碍物距离」 vs `inflation_radius + 车半宽`。
> 距离小于这个和，就别指望能规划出路径。

#### 与旧包的关系

两个包**通过 `/cmd_vel` + `/odom` + TF 标准接口互换，上层完全不用改**：

- 旧包：`/cmd_vel` → Gazebo 插件 → `/odom`
- 新包：`/cmd_vel` → 适配器 → `diff_drive_controller` → 硬件插件 → `/odom`

**这正体现了 ros2_control 的价值**：驱动层换成真车，上层（Nav2 / 任务层 / 安全层）一行不动。

> **涉及文件**：`src/my_robot_control/`（15 个文件）、
> `src/my_robot_tasks/my_robot_tasks/cmd_vel_adapter.py`