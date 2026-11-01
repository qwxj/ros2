# 两级 EKF：把两段 TF 都换成融合输出

> 本文是 README「已实现功能」的拆分子集，收录 **§31 ~ §32**（2 个小节）。
> 原始完整文档： [`docs/archive/README-2026-11-01-full.md`](../archive/README-2026-11-01-full.md) 第 1690~1903 行。

### 31. 两级 EKF：把两段 TF 都换成融合输出


#### 结构

```
map ──[ekf_global 28 Hz]──▶ odom ──[ekf_local 30 Hz]──▶ base_footprint ──[rsp]──▶ 传感器
        ↑                          ↑
   /amcl_pose + /odometry/local   /odom + /imu
```

| 级 | 节点 | `world_frame` | 输入 | 输出 TF | 替换了谁 |
|---|---|---|---|---|---|
| 1 | `ekf_local` | `odom` | `/odom` + `/imu` | **`odom → base_footprint`** | Gazebo 差速插件 |
| 2 | `ekf_global` | `map` | `/amcl_pose` + `/odometry/local` | **`map → odom`** | AMCL |

两级的分工：**第 1 级处理"轮式打滑"，第 2 级处理"激光定位跳变"。**

#### 第 ③ 步：单级融合到底有没有用？（反直觉的量化结论）

实验设计：原地转 `0.6 rad/s × 11 s`（理论 378°，实际 337°），
用 **Gazebo IMU 的 orientation 当真值**（EKF 只取 `vyaw`，不用 orientation，所以无循环依赖）。

| 来源 | 累计转角 | 相对 IMU 真值误差 | 过程中最大偏差 |
|---|---|---|---|
| IMU（真值） | 337.34° | — | — |
| **轮式 `/odom`** | 337.22° | **0.126°（0.037%）** | 0.631° |
| **融合 `/odometry/local`** | 336.53° | **0.807°（0.239%）** | 1.687° |

**融合反而更差 6.4 倍。**

**原因**：轮式 yaw 由轮子转角**几何推算**，无滑移时几乎无误差；
而融合掺入了 IMU 的 `vyaw`（角速度），**对角速度积分必然累积噪声与偏置** ——
即使噪声只有 0.0002 rad/s，积分 11 秒也会累积出 0.8°。

**结论（和 §25 / ⑫ 一致）**：
> **融合的价值不在"正常情况更准"，而在"轮式失效时不崩"。**
> 仿真里没有滑移 → 轮式已是最优估计，掺 IMU 是负收益；
> 只有注入 10% 打滑后，融合才体现出价值（消除 **91%** 偏差，见 ⑫）。

**这是一个诚实的负面结果，但它把"什么时候该用融合"说清楚了。**

#### 第 ④ 步：`ekf_global` 接管 `map → odom`

**配置要点**（`config/ekf_global.yaml`）：

```yaml
ekf_global:
  ros__parameters:
    world_frame: map          # 在 map 系内融合
    publish_tf: true          # 接管 map -> odom
    pose0: /amcl_pose         # 绝对位姿：只用 x, y, yaw
    pose0_config: [true,true,false, false,false,true, ...]
    odom0: /odometry/local    # 速度：只取 vx, vyaw（位置全 false，避免和 pose0 打架）
    odom0_config: [false×6, true,false,false, false,false,true, false×3]
```

**同时必须关掉 AMCL 的 TF 广播**，否则两个节点抢同一段 TF：

```yaml
# nav2_params.yaml
amcl:
  tf_broadcast: false   # 交给 ekf_global 发 map->odom
```

#### ⚠️ 踩到的坑：`ekf_global` 没有初始参考位姿

第一版实现后实测：

```
启动后  map->odom  y = -1.8896 m   ← 错了 1.89 m！
移动后  map->odom  y = -0.0257 m   ← 车一动、/amcl_pose 来了，自己修正回来
```

**原因**：`ekf_global` 在 22 秒启动，而 **`/amcl_pose` 受 `update_min_d: 0.25` /
`update_min_a: 0.2` 门控，车静止时不发**（见 §27）。于是它启动后的 80 秒里
**只有速度输入、没有绝对位姿**，只能从 0 开始积分 → `map→odom` 偏出 1.89 m。
**这期间全局代价地图是偏的，导航必然失败。**

**修法**：在 `ekf_global` 启动之前，用 AMCL 的 `/request_nomotion_update` 服务
**强制它做一次更新并发布 `/amcl_pose`**：

```python
force_amcl_update = ExecuteProcess(
    cmd=['bash', '-c',
         'for i in 1 2 3; do '
         '  ros2 service call /request_nomotion_update std_srvs/srv/Empty >/dev/null 2>&1; '
         '  sleep 1; '
         'done; echo "[bringup] 已强制 AMCL 更新 3 次"'],
    output='screen',
)

# 返回列表里：
TimerAction(period=20.0, actions=[force_amcl_update]),   # 先强制 AMCL 更新
TimerAction(period=25.0, actions=[ekf_global]),          # 再起 ekf_global
```

**修复后实测（车全程没动过）**：

```
map->odom : x=+0.0445  y=-0.0428  yaw=+0.03°
odom 位姿 : x=+0.2404  y=+0.0148  yaw=-0.0337
map->odom 平移量 = 0.0617 m       ✅ 正确
```

#### 两级 EKF 的最终验收数据

| 项 | 值 |
|---|---|
| `map → odom` | **28.2 Hz**（AMCL 本来只有 10 Hz → 确认已接管） |
| `odom → base_footprint` | **29.6 Hz** |
| `/odometry/global` | 28.2 Hz |
| `/odometry/local` | 29.6 Hz |
| 车静止时 `map→odom` 平移量 | **0.062 m** |
| Nav2 三个 lifecycle | 全部 `active` |
| 全局代价地图 | 110×81 @ 0.05 ✅ |

**三个 lifecycle 全 active + 两级 TF 都是连续高频。**

> ### 🔴 事后修正（重要）
> 上面的验收是在 `use_composition: True` 环境下做的，**那个环境里跨进程 DDS 发现是坏的** ——
> `ekf_global` 从来没收到过 `/amcl_pose`（见 §32）。所以当时看到的 `map→odom` 其实是**纯推算漂移**，
> 不是真正的融合结果。**关掉合成后重测**：`/amcl_pose` 的 publisher/ subscriber 都在，
> `map→odom ≈ (0.024, -0.015)`、`map→base ≈ odom 位姿`，这时才是真的在融合 AMCL。

#### 一个额外收获：`collision_monitor` 的 STOP 是**全向锁死**

实验中途车撞到了世界里的方块，激光最近 0.21 m（< `PolygonStop` 的 0.28 m），
`collision_monitor` 进入 STOP。此时：

- 前进被拦（预期）
- **倒车也被拦** —— `PolygonStop` 会把整条 `cmd_vel` 归零，**不分方向**

结果车被彻底锁死，只能重启仿真。**实操上要注意：撞墙后不能靠倒车自救。**

> **涉及文件**：`src/my_robot_description/config/ekf_global.yaml`、
> `src/my_robot_description/config/nav2_params.yaml`、
> `src/my_robot_description/launch/bringup_all.launch.py`

### 32. 第 ⑤ 步：单级 vs 两级的实测对比，以及一个把结论推翻的发现


#### 实验设计

用 `/initialpose` 让 AMCL **人为跳变 0.5 m**，测 `map→odom` 的**单步最大跳变量**。
这个做法安全、可重复、不用破坏运行中的节点。

#### 结果对比

| 指标 | **A 组：两级（`ekf_global` 发）** | **B 组：单级（AMCL 直接发）** |
|---|---|---|
| `map→odom` 采样率 | **24 Hz** | **9 Hz** |
| 总位移（目标 0.5 m） | 0.4078 m | 0.3897 m |
| **单步最大跳变** | **0.3533 m** | **0.3398 m** |
| 跳变后 3 秒内最大步 | 0.0010 m | 0.0000 m |

#### 结论：又一次推翻预设

**两级 EKF 并没有"平滑" AMCL 的硬跳变**（0.353 vs 0.340 m，几乎一样）。

原因：`robot_localization` 把 `pose0`（`/amcl_pose`）当**绝对量测直接吸收**。
要让跳变被摊平，需要显式调大 `pose0` 的协方差（降低信任）或用 `pose0_differential`，
**默认配置下不会自动平滑**。

**两级 EKF 的真实价值在别处**：`map→odom` 的**发布频率 9 Hz → 24 Hz（2.7 倍）**，
且是**持续高频**输出 —— 对依赖 `map→odom` 做插值的消费者（costmap、视觉投影）更友好。

#### 🔴 顺带发现一个会把整轮结论推翻的问题

做 ⑤ 的过程中发现 `/initialpose` **没有订阅者**，继续追下去：

```
$ ros2 topic info /amcl_pose -v        # 在 use_composition: True 下
publishers = 0     subscribers = 1 (/ekf_global)

$ 查询 AMCL 的所有 DDS 实体
AMCL 订阅的话题：   （空）
AMCL 发布的话题：   只有 parameter_events / rosout
```

**AMCL 在 DDS 里的实体全都不可见 → `/amcl_pose` 送不到 `ekf_global` → `ekf_global` 一直在纯推算。**

把 `use_composition` 改成 `False` 后立刻正常：

```
/amcl_pose: publishers=1 (/amcl)  subscribers=1 (/ekf_global)     ✅
/initialpose 订阅: ['amcl']                                       ✅

odom 位姿   : (0.225, 0.014)
map->odom   : (0.024, -0.015)      ✅ 接近 0
map->base   : (0.248, -0.001)      ✅ 与 odom 位姿吻合
```

**结论：`component_container_isolated` 会让容器内节点的 DDS 实体对其他进程不可见**
（至少在本机 Fast DDS + `UDPv4` + Humble 的组合下）。

#### 于是有了一个真实的权衡

| 配置 | CPU | 跨进程可见性 | 能否跑两级 EKF |
|---|---|---|---|
| `use_composition: True` | 好（`sy` 45%→15%，participant 13→4） | ❌ 容器内节点不可见 | ❌ 不能 |
| `use_composition: False` | 差（`sy` 回到 ~37%） | ✅ 正常 | ✅ 能 |

**当前选择 `False`** —— 因为功能正确性优先于 CPU。
`True` 的降载收益（§30）因此**没能保留**，这是遗留的技术债，下一步应该查：

1. `component_container`（非 isolated）是否也有同样问题？
2. 是 `UDPv4` 导致的吗？换成 SHM 或默认传输会不会好？
3. `rmw_cyclonedds_cpp` 是否没这个问题？

> **教训**：用「DDS 内省查不到实体」这种**间接证据**判断系统健康是不可靠的；
> 但**反过来，当内省显示"某节点没有任何实体"而它明明 active 时，一定要追到底**。
> 这一轮如果不追，§31 的结论就会一直错下去。

> **涉及文件**：`src/my_robot_description/launch/bringup_all.launch.py`（`use_composition`）
