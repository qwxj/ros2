# Nav2 导航与参数调优

> 本文是 README「已实现功能」的拆分子集，收录 **§19 ~ §22**（4 个小节）。
> 原始完整文档： [`docs/archive/README-2026-11-01-full.md`](../archive/README-2026-11-01-full.md) 第 368~622 行。

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
> `src/my_robot_description/maps/my_map.yaml`

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
  map:=$(ros2 pkg prefix my_robot_description)/share/my_robot_description/maps/my_map.yaml \
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
