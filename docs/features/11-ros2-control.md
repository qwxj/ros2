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

---

### 34. ros2_control 进阶：八项能力逐个落地

§33 把 ros2_control 的骨架跑通了，这一节在里面填了八件「真东西」。
每一项都附实测数据，不是「应该能行」。

#### ① 电机惯性：从「瞬时响应」到 `dω/dt = (τ − b·ω) / J`

原来的 `write()` 是 `velocity = clamp(cmd)` —— 指令一变速度立刻到位，没有任何动态过程。
现在改成力矩驱动的物理模型：

```cpp
// velocity 模式：速度环 P 控制 + 前馈，稳态无差但有动态滞后
torque = b * target + Kp * (target - velocity);
// effort 模式：力矩直接驱动
torque = clamp(effort_command, ±max_effort);

const double accel = (torque - b * velocity) / J;
velocity += accel * dt;
```

参数：`J = 0.5 kg·m²`、`b = 0.05`、`Kp = 2.0`、`max_effort = 5 N·m`
→ 理论时间常数 `τ = J/(b+Kp) = 244 ms`。

**实测阶跃响应**（目标 3.0 rad/s）：

| 指标 | 实测 | 理论 |
|---|---|---|
| 达到 63% 用时 | **300 ms** | 244 ms |
| 峰值 | 2.98 rad/s（无超调）| 3.0 |
| 前 200 ms 序列 | 0 → 0.1 → 0.2 → 0.3 → 0.4 | 一阶上升 |

**为什么这个改动重要**：没有它，`/odom` 是完美无噪声的，EKF 融合实验看起来「融合反而更差」
（见 §31）；有了动态滞后，融合才有真实的东西可滤。

#### ② 编码器量化：4096 线

```cpp
const double res = 2.0 * M_PI / encoder_cpr_;      // 0.001534 rad / 刻度
position_raw_ += velocity * dt;                    // 内部保持连续
position_ = std::round(position_raw_ / res) * res; // 对外只给离散刻度
```

**实测**：`position / res` 到最近整数的最大偏差 **4.55e-13** —— 说明对外暴露的确实是刻度整数倍。
（内部保留连续量，否则每步先量化再积分会系统性丢量。）

#### ③ IMU：`hardware_interface::SensorInterface`（第三种硬件类型）

ros2_control 有三种硬件类型：`SystemInterface`（读写）、`ActuatorInterface`（单执行器）、
**`SensorInterface`（只读，没有 command interface）**。

```xml
<ros2_control name="ImuSensor" type="sensor">
  <hardware><plugin>my_robot_control/ImuSensor</plugin></hardware>
  <sensor name="imu_sensor">
    <state_interface name="orientation.x"/>   <!-- 共 10 个 -->
    ...
  </sensor>
</ros2_control>
```

**它怎么知道机器人真的在转**：`DiffDriveHardware` 和 `ImuSensor` 在**同一进程的同一个 `.so`** 里，
用一个进程内单例 `ground_truth()` 传递「真实运动」：

```cpp
// DiffDriveHardware::read()  写入
gt.yaw_rate = (velocity_[1] - velocity_[0]) * wheel_radius / wheel_separation;
gt.yaw += gt.yaw_rate * dt;

// ImuSensor::read()  采样并加噪声
angular_velocity_[2] = gt.yaw_rate + bias_ + noise(rng_);
```

真实硬件里这个角色由物理世界承担，这里是「用一个进程内单例代替物理世界」。

**实测**：`/imu_sensor_broadcaster/imu` **99.99 Hz**；
静止时 vyaw 均值 **0.00492**（≈ 设定的 bias）、标准差 **0.01854**（≈ 设定的 σ=0.02）。

#### ④ 完整生命周期

| 回调 | 何时调 | 本实现 |
|---|---|---|
| `on_init` | 只调一次 | 读 `<param>`、**分配缓冲** |
| `on_configure` | 可反复 | 只 `fill(0)`，**不重新分配** |
| `on_activate` | 上电 | 清零 + 重置 ground_truth |
| `on_deactivate` | 下电 | 清速度/指令，保留位置 |
| `on_cleanup` | 关设备 | 只 `fill(0)`，**不 `clear()`** |
| `on_error` | 出错 | 强制停车 |

> ⚠️ **`on_init` 和 `on_configure` 的分工是有硬约束的**，见下面「坑 A」。

#### ⑤ `HW_IF_EFFORT`：状态 + 指令双向

```cpp
// 状态：力矩反馈（真实硬件是电流采样）——joint_states 里的 effort 不再是 NaN
out.emplace_back(name, HW_IF_EFFORT, &effort_[i]);
// 指令：motor_mode:=effort 时 write() 用它
out.emplace_back(name, HW_IF_EFFORT, &effort_command_[i]);
```

`ros2 control list_hardware_interfaces` 可以看到：

```
command interfaces
    left_wheel_joint/effort     [available] [unclaimed]   ← 暴露了，没被认领
    left_wheel_joint/velocity   [available] [claimed]     ← diff_drive_controller 认领了
```

**实测 effort 通路**（`motor_mode:=effort` + 切到 `manual_effort_controller`，发 0.30 N·m）：

| 指标 | 实测 | 理论 |
|---|---|---|
| 力矩反馈峰值 | **0.300 N·m** | 0.30 |
| 轮速峰值（2 S 后） | **1.098 rad/s** | `6·(1−e^(−2/10)) = 1.09` |
| 前 8 帧速度 | 0 → 0.01 → 0.02 → 0.03 → 0.04 | 一阶爬升 |

稳态 `τ/b = 0.30/0.05 = 6.0 rad/s`，时间常数 `J/b = 10 s` —— 完全吻合。

#### ⑥⑦ 安全链 + 两级 EKF

```
/cmd_vel ──watchdog──▶ /cmd_vel_safe ──collision_monitor──▶ /cmd_vel_final ──▶ 适配器
```

```
map ──[ekf_global 22 Hz，融合 /amcl_pose + /odometry/local]──▶ odom
odom ──[ekf_local 24 Hz，融合 /odom + IMU vyaw]──────────────▶ base_link
```

**关键的两行「让权」**：

```yaml
# controllers.yaml —— diff_drive_controller 不再发 TF
enable_odom_tf: false
```
```yaml
# nav2_params_standalone.yaml —— AMCL 不再发 TF
tf_broadcast: false
```

**实测**：`map→odom` 22.4 Hz、`odom→base_link` 23.6 Hz；
自主导航 (0,0)→(2.5,−1.5) `status=4 SUCCEEDED`，17.1 s，**距目标 0.100 m**
（比 §33 单级 EKF 的 0.244 m 更准 —— 因为 yaw 由 IMU 保住了）。

#### ⑧ `ros2 control switch_controllers` 运行时热切换

`diff_drive_controller` 和 `manual_velocity_controller` 争抢同一个 `velocity` 指令接口，
所以**只能有一个 active** —— 正好用来演示热切换。

```
$ ros2 control list_controllers
diff_drive_controller      active
manual_velocity_controller inactive
manual_effort_controller   inactive

$ ros2 control switch_controllers --strict \
    --deactivate diff_drive_controller --activate manual_velocity_controller
Successfully switched controllers
```

---

#### 坑 A：缓冲分配必须在 `on_init`，否则进程 SIGABRT

**现象**：硬件加载、configure、activate 全部成功，然后 `ros2_control_node` 直接崩：

```
terminate called after throwing an instance of 'std::runtime_error'
  what():  double hardware_interface::ReadOnlyHandle::get_value() const failed. value_ptr_ is null.
  #7 libdiff_drive_controller.so
  #8 controller_manager::ControllerManager::update(...)
exit code -6 (SIGABRT)
```

**根因**：`export_state_interfaces()` 是在 **`on_init` 阶段**被 `resource_manager` 调用的，
框架会**永久记住这些成员的地址**。而我把缓冲分配写在了 `on_configure`：

```cpp
// ❌ 错
on_init:      /* 什么都没分配 */
export_state_interfaces(): &position_[i]   // position_ 还是空 vector -> 越界/空指针
on_configure: position_.assign(n, 0.0);    // 重新分配 -> 之前给的地址全悬空
```

**正确做法**：

```cpp
// ✅ 对
on_init:                 position_.assign(n, 0.0);   // 只在这里分配
on_configure/on_cleanup: std::fill(...);             // 只 fill，绝不 assign/clear
```

> **一句话记住**：`export_*_interfaces` 给出去的是指针，
> 所以这些成员从 `on_init` 起就必须存在，且**直到进程结束都不能重新分配或释放**。

#### 坑 B：`switch_controllers` 的「假成功」

```
$ ros2 control switch_controllers --deactivate diff_drive_controller --activate manual_velocity_controller
Successfully switched controllers            ← 看着成功
$ ros2 control list_controllers
diff_drive_controller      inactive
(manual_velocity_controller 根本不在列表里)   ← 其实什么都没做
```

**根因**：`switch_controllers` 默认 `BEST_EFFORT` 严格度，**静默跳过未加载的控制器**。
没加载就跳过，然后照样打印成功。

**修法**：
1. launch 里用 `spawner <name> --inactive` **预加载**；
2. 调用时加 `--strict`，让它失败就报错。

#### 坑 C：`nohup setsid` 起的 launch，`pkill -INT` 杀不干净

残留第二个实例 → **所有话题频率翻倍**（`/scan` 20 Hz、`/odom` 98 Hz、`/odometry/local` 59 Hz）。
最坑的是：每一级单独看都「有数据」，看不出异常；**只有算一下「应该是多少」才会发现**。

**修法**：循环 `pkill` 直到 `pgrep` 计数为 0，再逐个按进程名兜底。

#### 坑 D：`imu_sensor_broadcaster` 的协方差默认是 0

`robot_localization` 看到 0 协方差会理解成「这个测量完美无缺」，
于是**无条件信任 IMU**，把陀螺偏置和噪声全部积分成 yaw 漂移。
必须显式给：

```yaml
static_covariance_angular_velocity: [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
```

而且这个值和 `ImuSensor` 实际注入的噪声 σ **必须匹配**（σ=0.01 → 方差 1e-4）。

#### 实测汇总（单实例，ERROR 0 条）

| 话题 | 频率 | 配置 |
|---|---|---|
| `/scan` | 10.000 Hz | 10 |
| `/joint_states` | 99.99 Hz | update_rate 100 |
| `/imu_sensor_broadcaster/imu` | 99.99 Hz | 100 |
| `/odom` | 50.01 Hz | publish_rate 50 |
| `/odometry/local` | 29.35 Hz | ekf_local 30 |
| `map→odom` | 22.4 Hz | ekf_global 30（CPU 受限）|
| `odom→base_link` | 23.6 Hz | ekf_local 30（CPU 受限）|