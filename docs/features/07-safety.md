# 第四层安全（collision_monitor）

> 本文是 README「已实现功能」的拆分子集，收录 **§26 ~ §26**（1 个小节）。
> 原始完整文档： [`docs/archive/README-2026-11-01-full.md`](../archive/README-2026-11-01-full.md) 第 886~1043 行。

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
