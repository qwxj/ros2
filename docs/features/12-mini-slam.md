# mini_slam：手写 SLAM / AMCL / EKF 最小实现

> 本包不依赖 `slam_toolbox`、`nav2_amcl`、`robot_localization`，
> 从零实现占据栅格建图、扫描匹配、位姿图、粒子滤波定位、EKF 状态估计和地图读写。
> 算法层零 ROS 依赖，43 项 gtest 用例 3 秒内跑完。

### 1. 为什么自己写一套

#### 1.1 直接调包留下的三个坑

在写这个包之前，项目里用的是 `slam_toolbox` + `nav2_amcl` + `robot_localization`。
能跑，但遇到问题时的排查路径基本是"试参数"：

| 现象 | 当时的做法 | 问题出在哪（事后才知道） |
|---|---|---|
| 走廊里定位飘 | 调 `max_particles`、`sigma_hit` | 其实是环境退化（perceptual aliasing），粒子再多也没用 |
| 静止时 AMCL 不更新 | 以为坏了，反复重启 | `update_min_d` / `update_min_a` 门控，设计如此 |
| 建图墙有重影 | 调 `minimum_travel_distance` | 扫描匹配的搜索范围不够，位姿没纠正过来 |
| 地图存出来未知区消失了 | 没注意 | `free_thresh` 把灰像素算成了空闲 |

**共同点**：出了问题只能猜，因为不知道里面在算什么。
能改的参数都试一遍，碰对了也不知道为什么对。

#### 1.2 三个目标

**目标一：算法层零 ROS 依赖。**

```
include/mini_slam/           <- 8 个头文件，一个 rclcpp 都没有
  pose2d.hpp                   位姿类型 + 角度归一化
  occupancy_grid.hpp           对数几率栅格 + Bresenham 光线投射
  distance_field.hpp           两遍 chamfer 距离变换
  scan_matcher.hpp             相关性扫描匹配
  particle_filter.hpp          粒子滤波（含 KLD + recovery）
  ekf.hpp                      5 维 EKF
  pose_graph.hpp               位姿图 + 松弛优化
  map_io.hpp                   PGM(P2/P5) + YAML 读写
```

带来的好处是**测试成本极低**：

```bash
colcon test --packages-select mini_slam
# 43 项用例，3 秒跑完，含端到端场景（从零建图、回环闭合、绑架恢复）
```

如果要起一整套仿真才能验证一个公式，就不会有人写测试，也不会有反馈。

**目标二：每个模块都对得上真实库的一部分。**

写这个包的目的不是替代它们，而是**搞懂它们**：

| `mini_slam` 模块 | 对应真实组件 | 简化了什么 |
|---|---|---|
| `ScanMatcher` | `slam_toolbox` 的 CSM | 暴力搜索，没有分支定界加速 |
| `PoseGraph` | `g2o` / `ceres` | Gauss-Seidel 松弛，不是稀疏高斯-牛顿 |
| `OccupancyGrid` | `slam_toolbox` 的 OccGrid | 单分辨率，没有多分辨率地图 |
| `ParticleFilter` | `nav2_amcl` | 没做 `update_min_d/a` 门控（在节点层简化掉了） |
| `Ekf2D` | `robot_localization` 的 `two_d_mode` | 5 维（没有 z / roll / pitch 及其速度） |
| `map_io` | `nav2_map_server` | 只支持 PGM，不支持 PNG |

学完再看真实库的源码，至少能对上号：知道 `transform_tolerance` 是干什么的、
知道 `kld_err` 在哪个公式里、知道 `recovery_alpha_slow` 在比较什么。

**目标三：端到端能跑，不是玩具。**

不能只停在"单元测试通过"。所以加了三个 ROS 节点把算法接起来：

| 节点 | 输入 | 输出 |
|---|---|---|
| `slam_mini_node` | `/scan` + `/odom` | `/map` + TF `map->odom` |
| `amcl_mini_node` | `/scan` + `/map` + `/odom` | `/amcl_pose` + TF `map->odom` |
| `ekf_node` | `/odom` + `/imu` | `/odometry/filtered` + TF `odom->base_link` |

**实测数据**（用项目里 `my_robot_control` 的真实 `/scan`，不是合成数据）：

```
建图：852 帧，扫描匹配 852 次成功、0 次退回
地图：200x200 @ 0.05 m，障碍 2.9% / 空闲 55.2% / 未知 41.8%
定位：换机位后跟踪误差 0.007 ~ 0.143 m
```

#### 1.3 明确不做什么

写在前面，避免期待错位：

| 不做 | 原因 |
|---|---|
| 代价地图膨胀（inflation） | 那是 Nav2 costmap 的职责，不属于 SLAM |
| 路径规划 | 同上 |
| 3D / 多分辨率地图 | 单分辨率已经够理解原理，多分辨率是工程加速手段 |
| 位姿图接进 SLAM 节点 | **目前只做了前端**，`PoseGraph` 只在单元测试里用（见 §7 局限） |
| 多线程 | 有意的。SLAM 的"匹配 → 建图 → 发 TF"必须严格有序，单线程天然没有竞态 |
| 完整的 beam model（z_short / z_max） | 只做了 `z_hit` + `z_rand` 两项，够用且足够说明问题 |

#### 1.4 一句话总结

> **用现成的库能做出功能，自己写一遍才能回答"为什么"。**
>
> 这个包的价值不在性能（它比 `slam_toolbox` 慢得多），
> 而在于每一行都能解释、每个参数都能说出所以然、每个失败都能定位到具体公式。

### 2. 四层架构

#### 2.1 分层总览

```
┌──────────────────────────────────────────────────────────────┐
│ ④ ROS 节点层（3 个 .cpp，676 行）                              │
│      ekf_node.cpp   amcl_mini_node.cpp   slam_mini_node.cpp   │
│      只做「ROS 消息 <-> 算法调用」的转换，不含任何算法           │
└───────────────────────────┬──────────────────────────────────┘
                            │ 调用
┌───────────────────────────▼──────────────────────────────────┐
│ ③ 算法层（6 个模块 + 1 个工具，纯 C++，1434 行）                │
│      OccupancyGrid   ScanMatcher   PoseGraph                  │
│      ParticleFilter  Ekf2D         map_io                     │
└───────────────────────────┬──────────────────────────────────┘
                            │ 依赖
┌───────────────────────────▼──────────────────────────────────┐
│ ② 基础层（84 行）                                              │
│      pose2d.hpp            位姿类型 + 角度归一化                │
│      distance_field.hpp    两遍 chamfer 距离变换                │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│ ① 测试层（与上面三层并行，1043 行 / 43 个用例）                  │
│      不依赖 ROS，colcon test 3 秒跑完                          │
└──────────────────────────────────────────────────────────────┘
```

#### 2.2 为什么这么切

**切开算法层和节点层，是为了让算法能被测。**

如果算法函数里直接拿 `rclcpp::Node`、直接读 `subscription`，
那验证一个公式就得起一套仿真 + 一张地图 + 一段 bag。

切开的直接收益：

| | 不切 | 切开 |
|---|---|---|
| 测一个公式 | 起仿真、放地图、录 bag | 造个 `OccupancyGrid`，调一次函数 |
| 单次反馈时间 | 30 秒以上 | 3 秒（43 个用例全部） |
| 能不能进 CI | 很难 | 一行 `colcon test` |

**基础层只有两个文件，是因为它们被多个算法共用：**

- `pose2d.hpp`：`ScanMatcher` / `ParticleFilter` / `PoseGraph` 都要 `Pose2D` 和 `wrapToPi`
- `distance_field.hpp`：`ScanMatcher`（打分）和 `ParticleFilter`（观测模型）都要距离场

如果把 `distance_field` 塞进 `occupancy_grid.hpp`，`OccupancyGrid` 就会同时承担
"存栅格"和"算距离变换"两个职责，测试也要跟着混在一起。

#### 2.3 文件级依赖图

```
                    pose2d.hpp ────────────────────┐
                    （无依赖）                      │
                                                   │
              occupancy_grid.hpp ────────────┐     │
              （只依赖 <cstdint> <vector>）     │     │
                    ▲                          │     │
                    │                          │     │
        distance_field.hpp                     │     │
                    ▲                          │     │
        ┌───────────┴──────────┐               │     │
        │                      │               │     │
  scan_matcher.hpp      particle_filter.hpp ◀──┴─────┤
        │                      │                     │
        │                      │      map_io.hpp ◀────┤
        │                      │      pose_graph.hpp ◀┘
        │                      │      ekf.hpp（只依赖 Eigen）
        ▼                      ▼            ▼
  slam_mini_node.cpp    amcl_mini_node.cpp   ekf_node.cpp
```

**几条实际规则**：

1. **算法头文件之间只有"向下"依赖**，没有环
   （`scan_matcher.hpp` 不会 include `particle_filter.hpp`）
2. **只有两个 `.cpp` 引用了 `distance_field.hpp`**：`scan_matcher.cpp`、`particle_filter.cpp`
3. **只有 `map_io.cpp` 引用 yaml-cpp**，其他算法文件不碰外部库
4. **`ekf.hpp` 只依赖 Eigen**，和栅格、地图完全无关

#### 2.4 一条 `/scan` 的完整旅程

以 `slam_mini_node` 为例（`slam_mini_node.cpp:onScan()`）：

```
/scan (10 Hz)  ─┐
/odom (100 Hz) ─┤
                ▼
   onOdom():  只记下 odom_pose_（微秒级，不做任何计算）
   onScan():  ← 主流程
                │
  ┌─────────────┴──────────────────────────────────────────────┐
  │ ① 由 odom 增量算「猜测位姿」                                 │
  │      dx, dy, dyaw = odom_pose − odom_prev                  │
  │      guess = pose_ ⊕ delta（把 odom 系的增量旋进 map 系）    │
  │                                                             │
  │ ② 激光抽稀：360 条 → 180 条                                 │
  │                                                             │
  │ ③ sm_.setMap(*map_)                                        │
  │      └→ buildDistanceField()   两遍 chamfer，O(W×H)         │
  │         每格 = 到最近障碍的距离（米）                         │
  │                                                             │
  │ ④ sm_.match(激光, guess, ±0.15m, ±0.15rad, 7 步)            │
  │      ├ 阶段 1 粗搜：7³ = 343 个候选                          │
  │      ├ 阶段 2 细搜：在最优点附近再 343 个                     │
  │      └ 每个候选：rawScore = (1/N)·Σ exp(−d²/2σ²)            │
  │                                                             │
  │ ⑤ 得分 ≥ 0.35 → 用匹配位姿；否则退回 odom 猜测                │
  │                                                             │
  │ ⑥ raycastIntoMap()  每条光束写进地图                         │
  │      └→ OccupancyGrid::raycast(x0,y0,x1,y1)                │
  │           ├ 沿途：updateFree()      L −= 0.7                │
  │           └ 终点：updateOccupied()  L += 0.9                │
  │         概率 p = 1 − 1/(1+e^L)，L 被 clamp 到 [−2, 3.5]      │
  │                                                             │
  │ ⑦ publishTf()  map→odom = (map→base) · inv(odom→base)       │
  │ ⑧ 每 publish_period 秒 publishMap() → /map                  │
  └─────────────────────────────────────────────────────────────┘
```

**关键点：③ 和 ④ 是纯算法层，⑤ 以后才是节点层的决策。**

如果把"得分阈值判定"写进 `ScanMatcher::match()` 里，
那这个阈值就没法在单元测试里单独验证 —— 这正是分层要避免的。

#### 2.5 三个节点的数据流

```
        /scan  ────────────┐
        /odom  ────────────┤
                           ▼
   ┌────────────────┬──────────────────┬────────────────┐
   │  ekf_node      │  amcl_mini_node  │  slam_mini_node│
   │  (Ekf2D)       │  (ParticleFilter)│  (ScanMatcher) │
   ├────────────────┼──────────────────┼────────────────┤
   │ /odom + /imu   │ /scan + /map     │ /scan + /odom  │
   │                │      + /odom     │                │
   └───────┬────────┴────────┬─────────┴────────┬───────┘
           │                 │                  │
           ▼                 ▼                  ▼
    /odometry/filtered   /amcl_pose          /map
    TF odom→base_link    TF map→odom         TF map→odom
```

**两个约束必须记住**（写代码时容易踩）：

| 约束 | 原因 |
|---|---|
| `ekf_node` 不能和 `robot_localization` 的 `ekf_local` 同时跑 | 都会发 `odom→base_link`，TF 会被随机覆盖 |
| `amcl_mini_node` 不能和 `slam_mini_node` 同时跑 | 都会发 `map→odom` |

这就是 ROS 的"一段 TF 只能有一个发布者"，和第 3 章（TF/URDF）里的规矩是同一个。

#### 2.6 代码量分布

| 层 | 文件数 | 行数 | 说明 |
|---|---|---|---|
| ROS 节点层 | 3 | 676 | 平均 225 行/节点 |
| 算法层 | 13 | 1434 | 8 头文件 + 6 实现（含 distance_field） |
| 基础层 | 3 | 84 | pose2d + distance_field |
| 测试层 | 6 | 1043 | 43 个用例 |

**测试代码 ≈ 算法代码的 73%** —— 这个比例不是刻意的，而是
"每修一个 bug 就补一个回归用例"积累出来的。

例：`test_occupancy_grid.cpp` 里有 3 个用例是回归测试：

| 用例 | 当时的 bug |
|---|---|
| `RaycastWallIsContinuous` | 光线投射在某些角度上会漏格 |
| `BoundaryRoundingIsStable` | `0.7/0.1 = 6.999...`，`floor` 得 6 而不是 7 |
| `BoundaryNotFallingToPreviousCell` | 上面的修复引入了新的边界问题 |

### 3. 七个模块逐个讲

每个模块固定四段：**解决什么问题 → 核心公式/算法 → 关键参数（真实默认值） → 对应真实库的哪部分**。

#### 3.1 `OccupancyGrid`：对数几率占据栅格

**解决什么问题**：把一帧帧激光"画"成一张概率地图，并且能区分"空闲"和"从没看过"。

**核心：对数几率（log-odds）累加**

用对数几率而不是直接存概率，是为了让多次观测变成**加法**（概率相乘 → 对数相加），
避免浮点下溢：

```
L = log( p / (1-p) )                     // 对数几率
p = 1 - 1/(1 + e^L) = 1/(1 + e^-L)       // 反解回概率
```

每次观测只做加减：

| 操作 | 代码 | 增量 | 含义 |
|---|---|---|---|
| 光束经过 | `updateFree()` | `L -= 0.7` | 这一格是空的 |
| 光束终点 | `updateOccupied()` | `L += 0.9` | 这一格有障碍 |
| 钳位 | `lo_occ_` / `lo_free_` | `L ∈ [-2.0, 3.5]` | 防止"一次误测就永久锁死" |

**为什么占据增量（0.9）> 空闲增量（0.7）**：障碍物是"存在性证据"，
一次打中就应该显著提高置信度；而"这里是空的"可能是穿过了一个未观测区域。

**`observed_` 位图：区分"空闲"和"没看过"**

只看 `L` 是不够的 —— `L = 0` 既可能是"从没观测过"，也可能是"观测过但正负抵消"。
所以另存一个 `uint8_t` 位图。这就是 `/map` 里 `-1`（未知）的来源：

```cpp
int8_t probability(int gx, int gy) const;   // 0~100；未观测返回 -1
```

**坐标转换：必须加 `kEps`**

```cpp
constexpr double kEps = 1e-9;
gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_ + kEps));
```

**为什么要有这个 1e-9**：`0.7 / 0.1` 在 IEEE 754 下等于 `6.99999999999999911`，
`floor` 得 6 而不是 7 —— 整条墙会错一格。

> 这个 bug 是靠一个"墙上有两个洞"的现象发现的：
> 20 cm 网格里，y=7 和 y=14 的墙格永远缺失。加 `kEps` 之后修复。
> 反向验证：把 `kEps` 设为 0，测试立刻重现 `[MISSING] wall cell (20, 7)`。

**光线投射：Bresenham**

从机器人位置到障碍点，沿线所有格子标空闲，终点标占据。
不用 DDA/浮点采样，是因为 Bresenham 全整数、边界严格一致。

| 参数 | 默认值 | 作用 |
|---|---|---|
| `lo_occ_` | `0.9` | 占据增量 |
| `lo_free_` | `-0.7` | 空闲增量 |
| `lo_min_` / `lo_max_` | `-2.0` / `3.5` | 钳位范围 |
| `resolution` | `0.05` m | 每格边长 |

**对应真实库**：`slam_toolbox` 的 `OccGrid`；`nav2_costmap_2d` 的静态层。

**测试**：8 个用例（坐标转换、log-odds 累加、光线投射、ASCII 建图、3 个浮点边界回归）。

---

#### 3.2 `distance_field`：两遍 chamfer 距离变换

**解决什么问题**：打分时需要知道"每格离最近的障碍有多远"。
暴力做法是每个格子遍历所有障碍，`O(W×H×N_障碍)`。

**算法：两遍扫描，`O(W×H)`**

```
① 初始化：障碍格 = 0，其余 = +∞
② 前向传播（左上 → 右下）：每格取 min(自己, 左+1, 上+1, 左上+√2, 右上+√2)
③ 后向传播（右下 → 左上）：对称再做一遍
④ 单位换算：格数 × resolution → 米
```

权重用 `1` 和 `√2`（对角线），最大误差约 2%。

**为什么要做这个替换**：在 400×400 的地图上（16 万格），
暴力版要几百毫秒，chamfer 只要零点几毫秒。而节点里**每帧都要重建一次**距离场。

| 参数 | 值 | 说明 |
|---|---|---|
| `kD1` | `1.0` | 正交邻居权重 |
| `kD2` | `1.41421356` | 对角邻居权重（√2） |

**对应真实库**：`nav2_costmap_2d` 的 `distance_transform`。

**测试**：无独立测试文件，由 `ScanMatcher`（3 个）和 `ParticleFilter`（9 个）间接覆盖。

---

#### 3.3 `ScanMatcher`：相关性扫描匹配

**解决什么问题**：odom 会漂，用当前激光在地图上"对齐"一下，纠正位姿。

**核心：在位姿空间做暴力搜索 + 距离场连续评分**

给定猜测位姿 `(x₀, y₀, yaw₀)`，在三维网格上撒候选点，每个候选算一个得分：

```
score = (1/N) · Σ exp( -d_i² / (2σ²) )
          i 遍历本条激光的 N 根光束
          d_i = 该光束端点离最近障碍的距离（查距离场，非负）
```

**为什么用距离场而不是"端点是否落在障碍格上"**：

落在格子上是 **0/1 的二值判断**，得分对位姿是阶跃的，没有梯度，没法优化。
用距离场就有了连续梯度 —— 位姿差 1 cm，得分也会平滑变化。

**两级搜索：由粗到细**

```
阶段 1  在 ±search_xy、±search_yaw 范围内撒 7³ = 343 个候选
        取最优
阶段 2  在最优点周围再撒 7³ = 343 个候选（步长缩小 1/4）
        取最优
```

总计 **686 个候选**。单级搜索的精度受限于步长，两级能在同样计算量下拿到更细的分辨率。

| 参数 | 默认值 | 作用 |
|---|---|---|
| `search_xy` | `0.20` m | 平移搜索半径 |
| `search_yaw` | `0.20` rad | 旋转搜索半径 |
| `steps` | `7` | 每维步数（7³ = 343） |
| `sigma` | `0.10` m | 评分的高斯宽度 |

**`sigma` 的物理含义**：容忍多大的对准误差。太小 → 得分过于尖锐，稍微差一点就没梯度；
太大 → 区分不出好位姿。0.10 m 是两格（0.05 m/格）。

**对应真实库**：`slam_toolbox` 的 CSM（Correlative Scan Matcher），
但真实实现有**分支定界加速**（用多分辨率金字塔剪枝），这里用暴力搜索。

**测试**：3 个用例 —— 得分区分度、已知偏移恢复（<1 cm）、从零建图。

---

#### 3.4 `ParticleFilter`：AMCL 的核心

**解决什么问题**：在一张已知地图上，从零确定自己的位置（全局定位），
或跟踪时防止漂移（位置跟踪）。

**三件套：预测 → 加权 → 重采样**

##### ① 预测：差分运动模型

把 odom 的位移分解成"转 → 直走 → 转"三段，每段加独立噪声：

```
delta_trans = hypot(dx, dy)
delta_rot1  = atan2(dy, dx)            // 车体系里的朝向角
delta_rot2  = dyaw - delta_rot1

var_rot1  = α1·rot1² + α2·trans²
var_trans = α3·trans² + α4·rot1² + α4·rot2²
var_rot2  = α1·rot2² + α2·trans²

rot1_hat  = rot1  + N(0, √var_rot1)
trans_hat = trans + N(0, √var_trans)
rot2_hat  = rot2  + N(0, √var_rot2)
```

`α1~α4` 就是"有多不信任里程计"，**换底盘类型差一个数量级**：

| 底盘 | α1 | α2 | α3 | α4 |
|---|---|---|---|---|
| 差速轮（干净地面） | 0.05 | 0.01 | 0.05 | 0.01 |
| 差速轮（打滑严重） | 0.20 | 0.05 | 0.20 | 0.05 |
| 履带 | 0.30 | 0.10 | 0.30 | 0.10 |
| 全向轮 | 0.02 | 0.05 | 0.05 | 0.10 |

##### ② 加权：beam model（不是纯高斯）

```
p(单根光束) = z_hit · exp(-d² / 2σ²) + z_rand · (1/range_max)

z_hit  = 0.95
z_rand = 0.05（对应 AMCL 的 laser_z_rand）
```

**为什么必须有 `z_rand` 这一项**（这是包开发过程中最大的一个坑）：

| | 一条完全打偏的光线 | 128 条全偏 |
|---|---|---|
| 纯高斯 | `log(exp(-22.2))` = **−22.2** | **−2842** |
| 加均匀项 | `log(0.05/8)` = **−5.07** | **−649** |

纯高斯下，权重会被**单个粒子独占**，其余粒子权重趋近 0 → 重采样后全变成同一个粒子的副本
→ **粒子瞬灭**，一旦丢失定位就永远回不来。

##### ③ 重采样：系统重采样

按累积权重等间隔抽样。相比"轮盘赌"（每个粒子独立随机抽），
系统重采样方差更小，同样粒子数下覆盖更均匀。

##### KLD 自适应粒子数

不用固定的 `max_particles`，而是按"粒子云占了几个格子"自适应：

```
① 把状态空间切成格子（默认 0.10 m × 0.10 m × 0.10 rad）
② 数粒子占了 k 个格子
③ n = (k-1)/(2ε) · (1 - 2/(9(k-1)) + √(2/(9(k-1))) · z)³
④ n 钳位到 [min_particles, max_particles]
```

`ε = kld_err`（允许误差），`z` 是 `kld_z` 对应的标准正态分位数（0.99 → 2.326）。

**实测效果**：

| 状态 | 占据格数 k | KLD 算出的 n |
|---|---|---|
| 刚初始化（很散） | 2850 | 29237 → 截到 3000 |
| 收敛后 | 1 | 100（下限） |

**收敛后粒子数从 3000 降到 100，省 30 倍算力。**

##### recovery：随机注入（默认关闭）

用慢/快两套平均似然检测"被绑架"：

```
w_slow += α_slow · (w_avg - w_slow)      α_slow = 0.001   长期
w_fast += α_fast · (w_avg - w_fast)      α_fast = 0.1     短期
注入概率 = max(0, 1 - w_fast / w_slow)
```

正常跟踪时 `w_fast ≈ w_slow` → 注入 ≈ 0；突然不匹配时 `w_fast` 迅速掉下来 → 注入接近 1。

**但这个包默认关闭它**（`enable_recovery: false`）。原因见 §5 的坑 2 —— 开着它 +
地图没建全，会让位姿乱跳 4.5 m。

| 参数 | 默认值 | 作用 |
|---|---|---|
| `max_particles` / `min_particles` | `2000` / `300` | 粒子数上下限 |
| `kld_err` / `kld_z` | `0.05` / `0.99` | KLD 参数 |
| `sigma_hit` | `0.15` m | 观测模型高斯宽度 |
| `max_dist` | `1.0` m | 距离场截断 |
| `max_beams` | `120` | 激光抽稀后的光束数 |
| `alpha1~4` | 见上表 | 运动模型噪声 |
| `enable_recovery` | `false` | 随机注入开关 |

**对应真实库**：`nav2_amcl`（`pf.c` + `pf_kdtree.c` + `laser_model`）。

**测试**：9 个用例（收敛、Neff、KLD 粒子数自适应、绑架注入、注入默认关闭）。

---

#### 3.5 `PoseGraph`：位姿图 + 回环闭合

**解决什么问题**：odom 累积漂移，走一圈回到起点时位姿已经偏了。
回环闭合就是"发现回到了老地方"，然后把整条轨迹拉回来。

**数据结构**

```
节点 = 位姿 (x, y, yaw)
边   = 【相对位姿观测】"在 from 的坐标系下，to 位于 (dx, dy, dyaw)"
```

**为什么用相对位姿而不是绝对位姿**：里程计和扫描匹配都只能测"相对上一帧变了多少"，
测不出绝对位姿。绝对位姿是优化**解出来**的，不是观测出来的。

**误差函数（在 `from` 的坐标系里）**

```
由 from 的当前位姿，推出 to 应该在哪：
  want_x   = from.x + cos(from.yaw)·dx - sin(from.yaw)·dy
  want_y   = from.y + sin(from.yaw)·dx + cos(from.yaw)·dy
  want_yaw = from.yaw + dyaw

误差 = want - 实际
  e_x = want_x - to.x
  e_y = want_y - to.y
  e_yaw = wrap(want_yaw - to.yaw)
```

总误差 = `Σ [ w_trans·(e_x² + e_y²) + w_rot·e_yaw² ]`

**优化：Gauss-Seidel 松弛（不是高斯-牛顿）**

```
重复 iterations（默认 200）次：
  对每条边：
    算出误差
    把误差的一半分给 to、一半分给 from
    （第 0 个节点 fixed，不动 —— 否则整张图会飘）
```

`alpha`（默认 0.6）是每轮移动的比例，小于 1 是为了稳定。

**这不是精确的最小二乘** —— 真实库用 `g2o` / `ceres` 做稀疏高斯-牛顿。
但松弛收敛、能明显把回环拉闭合，且 100 行能讲清楚。

**实测效果**（4 节点正方形 + 1 条回环边）：

| 指标 | 优化前 | 优化后 | 改善 |
|---|---|---|---|
| 总误差 | 0.0202 | 0.0020 | **10×** |
| 末端到起点距离 | 0.1176 m | 0.0023 m | **51×** |

**回环候选搜索**：只做"空间邻近筛选"（半径 + 最小间隔），
**没有用扫描匹配验证** —— 这是已知缺口，见 §7。

| 参数 | 默认值 | 作用 |
|---|---|---|
| `iterations` | `200` | 松弛轮数 |
| `alpha` | `0.6` | 每轮移动比例 |

**对应真实库**：`g2o` / `ceres` / `slam_toolbox` 的 `PoseGraph`。

**测试**：4 个用例（加节点加边、回环候选、优化闭环、固定节点不动）。

---

#### 3.6 `Ekf2D`：5 维扩展卡尔曼滤波

**解决什么问题**：融合轮速里程计和 IMU，得到比单传感器更平滑、更准的位姿。

**状态：5 维**

```
x = [x, y, yaw, vx, vyaw]ᵀ
```

对应 `robot_localization` 的 `two_d_mode`（关掉 z / roll / pitch 及其速度）。

**预测：匀速模型 + 圆周弧**

```
x'    = x + vx·cos(yaw)·dt          ← 沿朝向走
y'    = y + vx·sin(yaw)·dt
yaw'  = yaw + vyaw·dt
vx'   = vx                            ← 匀速假设
vyaw' = vyaw

F = ∂f/∂x （5×5 雅可比）
P = F·P·Fᵀ + Q
```

**两次更新**

| 方法 | 观测量 | 用哪个传感器 |
|---|---|---|
| `updateOdomTwist(vx, vyaw)` | `[vx, vyaw]` | `/odom` 的 twist |
| `updateYawRate(yaw_rate)` | `[vyaw]` | `/imu` 的角速度 |

标准 EKF 更新：

```
K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹
x = x + K·(z - H·x)
P = (I - K·H)·P
```

| 参数 | 默认值 | 作用 |
|---|---|---|
| `frequency` | `30.0` Hz | 预测步频率 |
| `sigma_v` / `sigma_w` | `0.05` / `0.05` | 速度观测噪声 |
| `sigma_yaw_rate` | `0.02` | IMU 角速度噪声 |

**一个实现细节**：`P_` 的初始值必须够大（`diagonal << 0.25, 0.25, 0.05, 0.1, 0.1`）。
如果初始化成 `1e-4`，EKF 会"不认账" —— 因为卡尔曼增益太小，
观测根本拉不动状态。这个坑是靠一个失败的测试发现的（`UpdatePosePullsPosition`）。

**对应真实库**：`robot_localization` 的 `ekf_node`（`two_d_mode: true`）。

**测试**：7 个用例（直线/圆周预测、IMU/位姿更新、协方差传播、角度跨 π 归一化）。

---

#### 3.7 `map_io`：地图读写

**解决什么问题**：SLAM 建出来的图要能存成文件，定位要能读回来。
格式必须和 `nav2_map_server` 完全一致，否则互相不认。

**文件格式：两个文件**

```
mini_map.yaml     元数据
mini_map.pgm      像素数据（P5 二进制，或 P2 ASCII）
```

**yaml 的 7 个字段**

```yaml
image: mini_map.pgm        # ★ 相对 yaml 所在目录，不是绝对路径
mode: trinary
resolution: 0.05
origin: [-5.0, -5.0, 0.0]  # 左下角的 (x, y, yaw)
negate: 0
occupied_thresh: 0.65
free_thresh: 0.15
```

**像素 → 占据度的规则（必须和 `nav2_map_server` 一致）**

```
occ = negate ? pixel/255 : (255 - pixel)/255

occ > occupied_thresh  → 100（占据）
occ < free_thresh      → 0  （空闲）
否则                    → -1（未知）
```

**反向：占据值 → 像素**

| 值 | 像素 | 显示 |
|---|---|---|
| 100（占据） | 0 | 黑 |
| 0（空闲） | 254 | 白 |
| −1（未知） | 205 | 灰 |

**`free_thresh` 决定"灰色算未知还是算空闲"**

```
pixel 205 → occ = (255-205)/255 = 0.196

free_thresh = 0.25  → 0.196 < 0.25 → 空闲   ← 项目原来那张图用的值
free_thresh = 0.15  → 0.196 > 0.15 → 未知   ← 更安全
```

**这不是纯技术细节，是安全隐患**：用 0.25 时，那张 110×81 的地图里
**一个未知格都没有**（实测：`map_server` 报 8698 空闲 / 212 占据 / **0 未知**），
意味着 20.7% 从没扫到过的地方被标成"可通行"，规划器会往那里走。

**PGM 的 y 轴是反的**

```
PGM 文件：第 0 行 = 图像顶部
栅格数组：gy = 0    = 世界 y 最小（底部）

读写都必须：  gy = height - 1 - row
```

**忘了翻转 → 地图上下颠倒**，而且从 ASCII 打印里**看不出来**（对称房间）。

| 参数 | 默认值 | 作用 |
|---|---|---|
| `negate` | `0` | 黑=占据（0）还是白=占据（1） |
| `occupied_thresh` | `0.65` | 占据阈值 |
| `free_thresh` | `0.25`（读）/ `0.15`（SLAM 存盘用） | 空闲阈值 |

**支持两种 PGM**：P5（二进制）和 P2（ASCII）。
本项目两张图恰好一张 P5（Gazebo 那套）、一张 P2（`my_robot_control` 那套），
所以两种都得支持。

**对应真实库**：`nav2_map_server` 的 `map_io.cpp`。

**测试**：6 个用例（像素规则、PGM 往返、真实地图对照 `map_server`、相对路径往返、`free_thresh` 实验、P2 格式）。

---

#### 3.8 七个模块一览

| 模块 | 一句话 | 行数 | 测试 |
|---|---|---|---|
| `OccupancyGrid` | 把激光画成地图 | 153 | 8 |
| `distance_field` | 每格离障碍多远 | 62 | （间接） |
| `ScanMatcher` | 用激光纠正位姿 | 135 | 3 |
| `ParticleFilter` | 在地图上定位 | 325 | 9 |
| `PoseGraph` | 消除累积漂移 | 119 | 4 |
| `Ekf2D` | 融合轮速 + IMU | 130 | 7 |
| `map_io` | 读写地图文件 | 259 | 6 |
| **合计** | | **1183** | **37** |

> `colcon test-result` 会报 **43 项**，比 37 多 6 —— 6 个测试二进制各额外出 1 条汇总条目。
> 真正的 gtest 用例数是 **37 个**。

### 4. 实测数据

数据分两类，**分开标注**，避免混淆：

| 类型 | 来源 | 可信度 |
|---|---|---|
| **算法单元测试** | gtest 里的合成场景（已知真值） | 可精确量化误差 |
| **端到端** | `my_robot_control` 的真实 `/scan` + `/odom` | 真实但无真值，用 odom 当参考 |

#### 4.1 测试环境

```
OS          Ubuntu 22.04 (VMware 虚拟机)
ROS 2       Humble
编译器      g++ (Ubuntu 11.4.0) -std=c++17 -O2 -Wall -Wextra
外部依赖    Eigen 3.4.0、yaml-cpp 0.7.0（均为 ROS 2 自带）
激光        128 线，角度增量 0.01745 rad，range_max 8.0 m
场景        my_robot_control 的 10 m × 8 m 仿真房间
```

#### 4.2 算法单元测试：37 个用例

| 测试文件 | 用例 | 耗时 | 覆盖 |
|---|---|---|---|
| `test_occupancy_grid` | 8 | < 1 ms | 坐标转换、log-odds、光线投射、3 个浮点边界回归 |
| `test_ekf` | 7 | 27 ms | 直线/圆周预测、IMU/位姿更新、协方差传播、角度跨 π |
| `test_pf` | 9 | 1982 ms | 收敛、Neff、KLD、绑架注入 |
| `test_sm` | 3 | 1046 ms | 得分区分度、偏移恢复、从零建图 |
| `test_pg` | 4 | 1 ms | 回环闭合、固定节点 |
| `test_map_io` | 6 | 20 ms | 像素规则、PGM 往返、真实地图对照、P2 |
| **合计** | **37** | **≈ 3.1 s** | |

> `colcon test-result` 会报 **43 项**，比 37 多 6 —— 6 个测试二进制各额外出 1 条汇总条目。

**测试里有 6 个是回归用例**，每个都对应一个真实修过的 bug：

| 用例 | 当时的 bug |
|---|---|
| `RaycastWallIsContinuous` | 光线投射在某些角度漏格 |
| `BoundaryRoundingIsStable` | `0.7/0.1 = 6.999...` → `floor` 得 6 |
| `BoundaryNotFallingToPreviousCell` | 上面的修复引入了新边界问题 |
| `UpdatePosePullsPosition` | `P_` 初始值太小，EKF 拉不动状态 |
| `PredictCircularArc` | **测试期望值写错**（90° 弧的终点距离是弦长 √2R，不是弧长） |
| `ConvergesToTruePose` | 房间内部格子概率为 −1，似然无区分度 |

#### 4.3 扫描匹配的量化效果

**（1）得分区分度** —— 真位姿的得分必须是最高

```
score(真位姿)     = 1.0000
score(偏移 0.3 m) = 0.6713
score(偏航 0.3 rad) = 0.3719
```

**（2）已知偏移恢复** —— 从偏掉的初值能否找回真值

```
真值           (2.000, 2.000, 0.300)
匹配结果       (2.009, 2.009, 0.293)
得分           0.9696
候选数         686（7³ 粗搜 + 7³ 细搜）
误差           ≈ 9 mm / 7 mrad
```

**（3）从零建图** —— 只用激光 + 带误差的 odom 能否重建房间

```
场景             odom 有 5% 尺度误差（每步走 0.21 m，真实 0.20 m）
纯 odom 走 8 步   漂移 0.080 m
匹配后最大位姿误差 0.0067 m          ← 误差缩小 26 倍
匹配成功帧数      8 / 8
建出的障碍格数    318
```

**这一项是整个包最有说服力的数据** —— 它证明扫描匹配真的在纠正漂移，
而不是"跟着 odom 走但看起来还行"。

#### 4.4 粒子滤波的量化效果

**（1）收敛能力**

```
初始散布     (2.0, 2.0, 0.0) ± (1.0 m, 0.6 rad)，3000 个粒子
收敛后        (1.903, 1.946, −0.003)
落在 0.15 m 内的粒子   3000 / 3000
```

**（2）KLD 自适应粒子数**

```
k = 2  →  n = 19.3
k = 10 →  n = 130.2
k = 50 →  n = 587.1
```

粒子数随占据格数单调增长，符合 Fox 2003 公式的预期。

**端到端效果**：

| 状态 | 占据格数 k | KLD 算出 n | 实际钳位后 |
|---|---|---|---|
| 刚初始化（很散） | 2850 | 29237 | 3000（上限） |
| 收敛后 | 1 | 100 | 100（下限） |

**收敛后粒子数从 3000 降到 100，省 30 倍算力。**

**（3）绑架恢复（随机注入）**

```
正常跟踪 5 帧后      注入概率 0.000000
喂 3 帧完全不匹配的激光  注入概率 1.000000
```

**3 帧（0.3 秒）就能检测到"被绑架"。**

#### 4.5 位姿图的量化效果

场景：2 m × 2 m 正方形闭环，odom 带 5% 尺度误差 + 每步 0.02 rad 偏转，
最后加一条回环边（`node4 → node0`，真实相对位姿为 `(0, 0, 0)`）。

| 指标 | 优化前 | 优化后 | 改善 |
|---|---|---|---|
| 总误差 | 0.0202 | 0.0020 | **10×** |
| `node4` 到起点距离 | 0.1176 m | 0.0023 m | **51×** |
| `node0`（fixed）| (0, 0) | (0, 0) | 不动 ✅ |

**各节点与真值的误差（优化后）**：

```
node0  0.0000 m     ← fixed，不动
node1  0.1001 m     ← 仍受 5% 尺度误差影响
node2  0.1327 m
node3  0.0947 m
node4  0.0023 m     ← 回环边直接约束的节点
```

**这个结果揭示了一个重要事实**：

> **回环只能修"闭环误差"（累积的位姿漂移），修不了"尺度误差"。**
> 中间节点还带着 10 cm 的尺度偏差，因为单条回环边无法反推每条 odom 边该多长。
> 真实 SLAM 用**多条回环互相约束**，或者用扫描匹配重新观测每条边。

#### 4.6 端到端：建图

用 `my_robot_control` 的真实 `/scan`（10 Hz）驱动 `slam_mini_node`：

| 指标 | 数值 |
|---|---|
| 处理帧数 | **852** |
| 扫描匹配成功 | **852** |
| 退回 odom 猜测 | **0** |
| 最终位姿（map 系）| (−1.67, 1.36, 2.89) |
| 同一时刻 odom | (−1.467, 1.352) |
| 两者差 | ≈ 0.20 m（这就是匹配纠正掉的漂移量）|

**地图统计**（200 × 200 @ 0.05 m）：

| 类别 | 格数 | 占比 |
|---|---|---|
| 障碍（黑） | — | 2.9% |
| 空闲（白） | — | 55.2% |
| 未知（灰） | — | 41.8% |

**未知 41.8%** 说明只走了一部分 —— 不是算法问题，是测试时机器人在
`collision_monitor` 的急停区内走不远（见 §5 的坑 5）。

#### 4.7 端到端：定位

流程：`slam_mini_node` 建图 → 存盘 → `map_server` 加载 → `amcl_mini_node` 定位。

| 时刻 | AMCL 估计 | odom | 差 |
|---|---|---|---|
| 起步 | (0.010, −0.013) | (0.000, 0.000) | **0.016 m** |
| 前进 ~0.9 m | (0.896, 0.006) | (0.898, 0.000) | **0.007 m** |
| 原地转向 | (0.869, 0.132) | (0.898, 0.000) | **0.135 m** |
| 再前进 | (1.055, 0.351) | (1.108, 0.217) | **0.143 m** |

**跟踪误差 0.007 ~ 0.143 m**（7 mm ~ 14 cm）。

转向和之后的采样点误差变大，是因为这两个采样点之间车被 `collision_monitor`
停住了（odom 也不再变化），AMCL 没有新的运动信息可用。

#### 4.8 地图格式与真实 `nav2_map_server` 的对照

这是验证 `map_io` 正确性的关键实验：**同一张图，我们读出来的判定
必须和真实 `map_server` 完全一致。**

**（1）Gazebo 那张（`my_map.pgm`，110×81，P5）**

| 类别 | 真实 `map_server` | `map_io` | 一致 |
|---|---|---|---|
| 空闲（0）| 8698 | 8698 | ✅ |
| 占据（100）| 212 | 212 | ✅ |
| 未知（−1）| **0** | **0** | ✅ |

**注意未知是 0** —— 因为那张图 `free_thresh: 0.25`，
而灰度像素 205 的占据度是 `(255−205)/255 = 0.196 < 0.25` → 被判成**空闲**。

**（2）把 `free_thresh` 改成 0.15 之后**

| 类别 | 格数 | 占比 |
|---|---|---|
| 未知（−1）| **1846** | 20.7% |
| 空闲（0）| 6852 | 76.9% |
| 占据（100）| 212 | 2.4% |

**这才是那张图"本来的样子"** —— 20.7% 的灰色区域应该算未知。
原来的配置把它们当成了可通行区域。

**（3）`my_robot_control` 那张（`map.pgm`，200×160，P2 ASCII）**

| 类别 | 格数 | 占比 |
|---|---|---|
| 空闲 | 27107 | 84.7% |
| 占据 | 4893 | 15.3% |
| 未知 | 0 | — |

这张图本身是纯二值的（只有 0 和 254），所以没有未知。

#### 4.9 性能

**单线程，无丢帧**：

| 指标 | 数值 |
|---|---|
| 进程线程数 | 11（1 主线程 + 10 个 Fast DDS IO 线程）|
| 主线程 CPU | 2.3% |
| `/scan` 频率 | 10.0 Hz |
| 实际处理帧率 | 10.0 Hz（852 帧 / 85 秒）|
| 丢帧 | 0 |

**为什么单线程够用**：单帧要做的三件事（距离场重建 + 686 候选打分 + 360 条光线投射）
加起来远小于 100 ms 的周期。

**如果哪天不够用，优先改算法而不是加线程**：

| 优化方向 | 收益 |
|---|---|
| `setMap()` 只在关键帧重建，不每帧都建 | 省掉整个距离变换 |
| 激光抽稀 128 → 64 条 | 打分量减半 |
| `search_steps` 7 → 5（343 → 125 候选）| 快 2.7 倍 |

**扫描匹配是纯 CPU 密集，多线程只会让多个回调抢同一个核。**
并发解决的是 IO 等待，不是 CPU 密集 —— 所以这个包有意保持单线程。

### 5. 六个坑：现象 → 根因 → 修法

这一节是全篇最有价值的部分 —— 每个坑都是**真实踩过、真实修过、真实验证过**的。

#### 5.1 坑一：观测模型缺 `z_rand`，粒子瞬灭

**现象**

用真实地图跑 AMCL，前进阶段正常，一转向就丢：

```
起步    AMCL( 0.09,  0.01)   odom(0.00, 0.00)   差 0.09 m   ✅
前进1   AMCL( 0.86,  0.03)   odom(0.76, 0.00)   差 0.10 m   ✅
转向    AMCL( 0.02, -0.01)   odom(0.76, 0.00)   差 0.74 m   ❌ 跳回原点
前进2   AMCL(-0.10,  0.18)   odom(0.99, 0.20)   差 1.16 m   ❌ 彻底丢
```

**根因**

最初的观测模型是纯高斯：

```cpp
log_l += -d * d / (2.0 * sigma_hit * sigma_hit);   // d 被 clamp 到 max_dist=1.0
```

一条**完全打偏**的光线（`d = 1.0`，`sigma_hit = 0.15`）：

```
贡献 = -1.0 / (2 × 0.15²) = -22.2 nats
```

128 条光线全偏 → **−2842 nats**；而完全正确只有 0。

这个尖峰导致**权重被单个粒子独占**，其余粒子权重趋近 0。
重采样之后，整个粒子云变成那一个粒子的副本 → **粒子瞬灭**，再也回不来。

**修法**

改成 AMCL 的 beam model（只保留 `z_hit` + `z_rand` 两项）：

```cpp
const double z_hit = 0.95;
const double z_rand = 0.05;                    // 对应 AMCL 的 laser_z_rand
const double rand_term = z_rand / range_max;

const double p_hit = std::exp(-d * d * inv_2sigma2);
log_l += std::log(z_hit * p_hit + rand_term);   // ← 加均匀项兜底
```

修完之后一条打偏的光线只值 `log(0.05/8) = -5.07`，128 条 −649（而不是 −2842）。

| | 单条最差 | 128 条最差 |
|---|---|---|
| 纯高斯 | −22.2 | −2842 |
| 加 `z_rand` | −5.07 | −649 |

**教训**

> **似然模型不是"越尖锐越好"。**
> 太尖 → 权重被单个粒子独占 → 粒子瞬灭 → 一旦丢失就永远回不来。
> `z_rand`（均匀项）的作用不是"提高精度"，而是**给滤波器留一条退路**。

---

#### 5.2 坑二：`recovery` 默认开着，位姿乱跳 4.5 米

**现象**

用 `slam_mini_node` 自己建的地图做定位，位姿在图上乱跳：

```
起步    AMCL( 0.176, -0.251)   odom(0.000, 0.000)   差 0.31 m
前进1   AMCL( 4.833,  1.239)   odom(0.556, 0.000)   差 4.45 m   ❌ 飞了
转向    AMCL(-1.120, -0.041)   odom(0.556, 0.000)   差 1.70 m
前进2   AMCL(-1.046, -1.030)   odom(0.539, 0.964)   差 2.55 m
```

坐标在整张图范围（±5 m）内随机跳，说明**粒子被撒到了全图**。

**根因**

`recovery`（随机注入）当时是开着的。它的判据是：

```
w_diff = max(0, 1 - w_fast / w_slow)
```

而 `w_fast` / `w_slow` 用的是**绝对似然的平均值**。

自己建的地图只有 55% 覆盖。机器人一走进未建图区域：

```
所有粒子的绝对似然一起下降
    ↓
w_fast 迅速掉下来，w_slow 还很高
    ↓
误判成"被绑架了"
    ↓
w_diff ≈ 1.0 → 满功率撒 2000 个随机粒子
```

**修法**

**默认关闭**（`enable_recovery: false`）。要用时显式打开：

```bash
ros2 launch mini_slam amcl_mini.launch.py enable_recovery:=true
```

**中间还试过一个错误的方向**（值得记下来）：

> 曾经想"改用归一化后的平均权重"，因为归一化权重是尺度无关的。
> 改完发现：**归一化后平均权重恒等于 1/N**（权重和永远是 1），是个常数，
> 永远不触发注入 —— 反而把绑架恢复测试搞挂了（`p_bad = 2.2e-16`）。
> 已回退。

**教训**

> **`recovery` 是给"绑架机器人"用的，不是给"正常定位"用的。**
> 看起来"更健壮"的功能，配上一个不完整的输入（只建了 55% 的地图），
> 会变成一场灾难。**默认值必须是保守的。**

---

#### 5.3 坑三：`predict()` 的坐标系语义

**现象**

转向之后位姿跳到原点附近，看起来像"失锁"。

**根因**

`ParticleFilter::predict(dx, dy, dyaw)` 内部把 `(dx, dy)` 当成**车体系**的增量：

```cpp
const double delta_trans = std::hypot(dx, dy);
delta_rot1 = std::atan2(dy, dx);                        // ← 车体系里的方位角
p.pose.x += trans_hat * std::cos(p.pose.yaw + rot1_hat); // ← 再旋到世界系
```

但节点传进去的是 **odom 系**的增量：

```cpp
pending_dx_ += now_pose.x - last_odom_.x;    // ← odom 系，不是车体系
```

**起步时 `yaw = 0`，两种语义重合** —— 所以前进阶段看起来完全正常。
一旦转向，整段位移就被**多旋转了一个 `yaw`**。

真实 AMCL 在 `differential_motion_model.cpp` 里是这么处理的：

```cpp
delta_bearing = atan2(delta.y, delta.x) - old_odom_yaw;
//               ↑ odom 系方位角       ↑ 减去上一刻 odom 的朝向 = 转成车体系
```

**修法**

在节点侧把 odom 增量转到车体系：

```cpp
const double c = std::cos(last_odom_.yaw);
const double s = std::sin(last_odom_.yaw);
const double body_dx =  c * pending_dx_ + s * pending_dy_;
const double body_dy = -s * pending_dx_ + c * pending_dy_;

pf_->predict(body_dx, body_dy, pending_dyaw_);
```

**教训**

> **"变量叫什么"不等于"它在哪个坐标系"。**
> 起步位置恰好是原点、朝向恰好是 0 的时候，坐标系错误会**完美隐藏**。
> 这类 bug 只在特定初始条件下暴露 —— 所以单元测试里必须有一个**非零初始位姿**的用例。

---

#### 5.4 坑四：PGM 有 P2 和 P5 两种格式

**现象**

```bash
ros2 launch mini_slam debug_map.launch.py map:=/home/wxj/ros2/src/my_robot_control/maps/map.yaml
# ERROR: not a P5 pgm (only binary P5 supported)
```

但这张图 `nav2_map_server` 能正常加载。

**根因**

PGM 有两种编码：

| 格式 | 编码 | 本项目里的例子 |
|---|---|---|
| **P5** | 二进制 | `my_robot_description/maps/my_map.pgm` |
| **P2** | ASCII（文本数字） | `my_robot_control/maps/map.pgm` |

```bash
$ head -c 24 src/my_robot_control/maps/map.pgm | xxd
00000000: 5032 0a23 2047 656e 6572 6174 6564 206f  P2.# Generated o
         ↑↑
        ASCII
```

`map_io` 最初只实现了 P5，遇到 P2 直接报错。

**修法**

`readPgm()` 同时支持两种，读头部时判断魔数：

```cpp
hdr.ascii = (buf[i + 1] == '2');    // P2 = ASCII，P5 = 二进制
```

- P5：直接按字节拷贝
- P2：跳过注释和空白，逐个解析十进制整数

写盘统一用 P5（体积小、解析快）。

**教训**

> **"同一个文件格式"不等于"同一种编码"。**
> 两张图在自己项目里各用一种编码，只有两套都跑一遍才会发现。

---

#### 5.5 坑五：`collision_monitor` 挡住了建图

**现象**

命令 `5 s × 0.25 m/s = 1.25 m`，但 `odom` 只走到 `0.677 m`：

```
前进 5s (0.25 m/s)   → odom x: 0.677    ← 只走了 54%
```

建出来的地图 **未知区域占 41.8%**。

**根因**

`robot.log` 里：

```
[collision_monitor]: Robot to stop due to PolygonStop polygon
```

`collision_monitor` 的 `PolygonStop` 是**前方 0.28 m** 的急停区。
机器人一起步就贴到了障碍物 → 被反复急停。

**修法**

**先倒车脱离墙边**，再开始建图：

```bash
ros2 topic pub -r 20 --times 80 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.20}}"
```

**更好的修法**（未做）：建图时把 `PolygonStop` 换成 `PolygonSlowdown`，
或者把建图起点选在场地中央。

**顺便发现的一件事（很重要）**

曾经想"直接杀掉 `collision_monitor` 让它自由跑"，结果**车一步都不动**：

```
杀了 collision_monitor 后：
  odom x: 0.0（全程）
```

因为安全链是**串联**的：

```
/cmd_vel → watchdog → /cmd_vel_safe → collision_monitor → /cmd_vel_final → 差速插件
                                          ↑ 杀掉这个
                                    /cmd_vel_final 没人发
                                      → 车永远不动
```

**这反过来验证了串联安全层的设计是有效的**：任何一环挂掉，车都不会动（fail-safe）。

**教训**

> **安全层不是"障碍"，是"正确的障碍"。**
> 遇到它挡住你，先想清楚"它为什么挡"（前方 0.28 m 有东西），
> 而不是直接关掉它 —— 关掉之后你会发现串联链路直接断了。

---

#### 5.6 坑六：重复起 `map_server`，静默无报错

**现象**

同时启动 `map_server.launch.py`（当时还叫这个名字）和 `amcl_mini.launch.py`：

```
$ ros2 node list
  /amcl_mini
  /map_server        ← 第 1 个
  /map_server        ← 第 2 个
  /map_server        ← 第 3 个（更早的没清干净）
  /lifecycle_manager_map
  /lifecycle_manager_map

$ ros2 topic info /map
  Publisher count: 3        ← 三个节点抢着发同一个话题
  Subscription count: 1

冲突警告：无
```

**根因**

`amcl_mini.launch.py` **内部已经起了 `map_server` + `lifecycle_manager`**。
再起 `map_server.launch.py` 就是重复。

而且 **ROS 2 完全不会拦你** —— 同名节点不报错，重复发同一个话题也不报错。

**后果**：

| 问题 | 说明 |
|---|---|
| 三份 `/map` | AMCL 收到的是混合数据 |
| `transient_local` QoS | 晚加入的订阅者会收到**所有三个**的历史消息 |
| 三张图不同时 | 定位彻底乱，而且**查不出原因** |
| `ros2 param get /map_server` | 随机命中一个，改了半天不知道改的谁 |

**修法**

**两件事：**

1. 把 `map_server.launch.py` **改名 `debug_map.launch.py`**，
   并在 docstring 里明确写"这是调试用的，平时定位不用起"。

2. 文档里加一条自查命令：

```bash
ros2 node list | grep -c map_server      # 必须是 1
ros2 topic info /map | grep Publisher    # 必须是 1
```

**教训**

> **ROS 2 对"重复"非常宽容 —— 重名节点不报错，重复发话题不报错。**
> 所以"能跑起来"不等于"跑对了"。判断标准必须是：
> **`ros2 topic info` 的 Publisher count 是 1。**

---

#### 5.7 其他小坑（不展开）

| 坑 | 现象 | 一句话 |
|---|---|---|
| `kEps` 浮点边界 | 墙上莫名出现两个洞 | `0.7/0.1 = 6.999...`，`floor` 得 6 |
| `LifecycleNode` 缺 `namespace` | `__init__() missing 1 required keyword-only argument` | Humble 的 `LifecycleNode` 必须显式传 `namespace=''` |
| `map:=''` 传空串 | `malformed launch argument 'map:='` | bash 会吃掉空串，ROS 2 launch 不接受空值 → 改用 `none` 哨兵值 |
| `colcon build` 不删已移除的文件 | 删了源码里的 `demo_map`，install 里还在 | 必须 `rm -rf build/<pkg> install/<pkg>` 再编译 |
| `P_` 初始值太小 | EKF 观测拉不动状态 | `P_ = diag(1e-4)` → 改成 `diag(0.25, 0.25, 0.05, 0.1, 0.1)` |
| 测试期望值写错 | `PredictCircularArc` 失败（r=1.40 vs 期望 1.0）| 90° 弧的终点距离是**弦长** `2R·sin(θ/2) = √2`，不是弧长 |

### 6. 和真实 SLAM 栈的对照

这个包**不是**要替代生产库，而是搞懂它们。所以这一节逐个功能点对齐，
并标出「实现了 / 简化了 / 没做」。

#### 6.1 对应 `slam_toolbox`

| `slam_toolbox` 的功能 | 本包 | 对应文件 | 简化了什么 |
|---|---|---|---|
| 激光扫描匹配（CSM） | ✅ | `scan_matcher.*` | 暴力搜索，**没有分支定界加速** |
| 占据栅格建图 | ✅ | `occupancy_grid.*` | 单分辨率，无多分辨率金字塔 |
| 发布 `/map` | ✅ | `slam_mini_node.cpp` | 固定周期发，不是增量发 |
| 发布 TF `map→odom` | ✅ | 同上 | — |
| 地图存盘（PGM + YAML） | ✅ | `map_io.*` | 只支持 PGM，不支持 PNG |
| 位姿图 | 🟡 | `pose_graph.*` | **有，但没接进节点** |
| 回环检测 | 🟡 | `findLoopCandidates()` | 只按空间距离筛，**没有扫描匹配验证** |
| 图优化 | 🟡 | `optimize()` | Gauss-Seidel 松弛，不是稀疏高斯-牛顿 |
| 分支定界加速 | ❌ | — | 暴力搜索，慢约 10 倍 |
| 多分辨率地图 | ❌ | — | 单分辨率 |
| 生命周期管理 | ❌ | — | 普通 `rclcpp::Node` |
| `.posegraph` 序列化 / 续建 | ❌ | — | — |
| 地图加载后继续建图 | ❌ | — | — |

**最关键的差距是位姿图没接进节点** —— 也就是说现在跑的是**只有前端的 SLAM**，
累积漂移完全没有后端来消除。这是 §7 局限里的第一条。

#### 6.2 对应 `nav2_amcl`

| `nav2_amcl` 的功能 | 本包 | 对应文件 | 简化了什么 |
|---|---|---|---|
| 粒子滤波 | ✅ | `particle_filter.*` | — |
| 差分运动模型 `alpha1~4` | ✅ | `predict()` | — |
| 观测模型（beam model） | 🟡 | `logLikelihood()` | 只有 `z_hit` + `z_rand`，**缺 `z_short` / `z_max`** |
| 似然场（距离场） | 🟡 | `distance_field.*` | chamfer **近似**，最大 8.24% 误差（AMCL 用精确 Voronoi 波前） |
| KLD 自适应粒子数 | ✅ | `kldSampleSize()` | bins 尺寸写死 0.10，没做自适应 |
| recovery 随机注入 | ✅ | `injectionProbability()` | **默认关闭** |
| 系统重采样 | ✅ | `resample()` | — |
| 激光抽稀 `max_beams` | ✅ | 节点里做 | — |
| `/initialpose` 重定位 | ✅ | `onInitialPose()` | — |
| `/amcl_pose` | ✅ | `publish()` | **协方差写死**（应为粒子云方差）|
| TF `map→odom` | ✅ | `publishTf()` | 可关（`publish_tf`） |
| `update_min_d` / `update_min_a` 门控 | ❌ | — | 每帧都更新 |
| `set_map` 服务 | ❌ | — | 只靠订阅 `/map` |
| 多激光 / 传感器融合 | ❌ | — | 单激光 |
| 全局定位模式 | ❌ | — | 必须给初始位姿 |

**最需要补的两条**：

1. **协方差写死** → 无法接进两级 EKF（EKF 靠协方差决定信任度，写死就退化成固定加权）
2. **距离场用 chamfer 近似** → 打分够用，但不能用于安全距离判断

#### 6.3 对应 `robot_localization` 与 `nav2_map_server`

| 真实组件 | 本包 | 覆盖度 |
|---|---|---|
| `robot_localization` 的 `two_d_mode`（5 维状态）| `ekf.hpp` + `ekf.cpp` | 🟡 5 维子集，没有 z / roll / pitch |
| 多传感器融合（`odom0`, `imu0`, `pose0`...）| 固定两路（odom twist + IMU yaw rate）| 🟡 简化 |
| 两级 EKF（local / global）| `ekf_node` 只实现了**一级** | 🟡 单级 |
| `nav2_map_server` 的 PGM/YAML 读写 | `map_io.*` | ✅ **判定规则完全一致**（实测对照）|
| 生命周期服务 | ❌ | 普通节点 |

#### 6.4 两级 EKF 为什么 `/odometry/local` 只传速度

**这是本项目里最容易混的一个设计点。**

先看两个「方向相反」的顺序：

```
【TF 树的层级】global 在上
  map ──(ekf_global 发)──▶ odom ──(ekf_local 发)──▶ base_link
   ↑                       ↑                       ↑
 最"全局"                中间                   最"局部"

【数据流的顺序】local 先算
  /odom + /imu ──▶ ekf_local ──/odometry/local──▶ ekf_global ──▶ map→odom
                                                    ↑
                                          吃的是 local 的输出
```

**这两个方向相反是正常的**：

| | `ekf_local` | `ekf_global` |
|---|---|---|
| 要知道什么 | 只知道**自己的相对运动** | 要知道**在 map 里的绝对位置** |
| 需要地图吗 | **不需要** | **必须** |
| 输出 | `odom→base_link` | `map→odom` |
| 漂移 | 会漂 | 不漂（AMCL 提供绝对参考）|

##### 关键：中间只传「速度」，不传「位姿」

项目里的 `ekf_global.yaml`：

```yaml
odom0: /odometry/local
odom0_config: [false, false, false,     # x, y, z           ← 位姿，不用
               false, false, false,     # roll, pitch, yaw  ← 位姿，不用
               true,  false, false,     # vx                ← 速度，用
               false, false, true,      # vyaw              ← 速度，用
               false, false, false]     # 加速度，不用
```

`/odometry/local` 消息里**既有位姿也有速度**，但 `ekf_global` 只取 `vx` 和 `vyaw`。

**为什么位姿不能跨级用，速度可以**：

| | 位姿 | 速度 |
|---|---|---|
| 带坐标系吗 | ✅ 带（`/odometry/local` 的位姿在 **odom** 系）| ❌ 不带（`vx` 沿车头、`vyaw` 绕 z 轴）|
| `ekf_global` 工作在 | **map** 系 | 同一个（车体系）|
| 能直接用吗 | ❌ 差一个 `map→odom` 变换 | ✅ 能用 |

**而 `map→odom` 正是 `ekf_global` 要算的输出**：

```
想用 /odometry/local 的位姿：
   odom 系位姿 ──需要──▶ map→odom ──▶ map 系位姿
                            ↑
                      这是我要算的输出啊！
                      → 循环依赖
```

**速度没有坐标系问题**，因为它描述"沿车头方向多快"，在哪个系里都一样。

##### 一句话

> **局部的先算，把「速度」往上送（速度不带坐标系）；
> 全局的后算，用绝对位姿 + 局部速度，解出 `map→odom`。
> 中间只传速度，不传位姿 —— 否则形成循环依赖。**

#### 6.5 为什么 SLAM 和 AMCL 必须互斥

##### 不是设计缺陷，是 TF 规则

```
map ──(map→odom)──▶ odom ──(odom→base_link)──▶ base_link
        ▲
        │ 谁算这条？
        ├─ slam_mini_node：扫描匹配算出来的
        └─ amcl_mini_node：粒子滤波算出来的
```

**一条 TF 只能有一个发布者。** 两个都发 → 下游（Nav2 / RViz）看到的位姿会在两种估计之间随机跳变。

这和项目里 `transform_tolerance`、`tf_broadcast: false` 是同一个规矩：
**一段 TF 一个发布者**。

##### 证据：项目里的真实栈也是这么用的

```bash
$ grep -rln 'slam_toolbox' --include=*.py --include=*.yaml src/
  src/my_robot_description/config/slam_toolbox.yaml

$ grep -rln 'amcl' --include=*.py --include=*.yaml src/
  src/my_robot_control/config/nav2_params_standalone.yaml
  src/my_robot_description/config/nav2_params.yaml
  ...

有没有任何文件同时启动两者？→ 没有
```

**Nav2 官方的用法就是二选一。**

##### 它们是「两个阶段」，不是「两个组件」

```
阶段 A：第一次去某个场地
   slam_mini_node → 边定位边建图 → 产出 mini_map.yaml + .pgm

阶段 B：地图有了，以后每次去
   amcl_mini_node → 加载 A 阶段的地图 → 只定位，不建图
```

**「边画地图边用上次的地图找自己」逻辑上就矛盾。**

类比：
- `slam_mini_node` = 第一次去陌生商场，边走边画地图
- `amcl_mini_node` = 第二次去，拿着上次画的地图找自己在哪

##### 真实产品里的第三种做法

| 产品 / 库 | 怎么做 |
|---|---|
| **Cartographer** | `pure_localization: true` —— 加载 `.pbstream` 后纯定位，不再建图 |
| 仓储机器人 | 界面上有「建图模式 / 运行模式」开关 |
| 扫地机器人 | 建图建完自动切定位 |

**即「一个节点两种模式」** —— 这是本包可以改进的方向（见 §7）。

#### 6.6 SLAM 与 AMCL 的定义与区别

| 缩写 | 全称 | 中文 |
|---|---|---|
| **SLAM** | **S**imultaneous **L**ocalization **A**nd **M**apping | 同时定位与建图 |
| **AMCL** | **A**daptive **M**onte **C**arlo **L**ocalization | 自适应蒙特卡洛定位 |

**名字里就写着最大区别**：SLAM 是「定位 **和** 建图」两件事，AMCL 只做定位。

##### SLAM 的核心矛盾：鸡生蛋

```
要建图，我得知道自己在哪          ← 位姿
要知道自己在哪，我得有一张地图     ← 地图

     位姿 ⟷ 地图    互相依赖，谁都算不出来
```

**这就是 SLAM 存在的意义**：不是"先定位再建图"，而是**联合估计**：

```
求  argmax  P( x_{1:t} , m | z_{1:t} , u_{1:t} )
     x,m
```

**注意是「联合后验」** —— 轨迹和地图一起估。规模是 `t × W × H` 维，直接解不可能，
工程上拆成**前端**（扫描匹配 + 建图）和**后端**（回环检测 + 图优化）。

##### AMCL 的数学形式

```
地图 m 已知且固定

求  argmax  P( x_t | z_{1:t} , u_{1:t} , m )
```

对比 SLAM：**`m` 从"待求"变成"已知"**，`x_{1:t}` 从"整条轨迹"变成"当前位姿"。

##### 为什么两者用不同算法

| | 算法 | 为什么 |
|---|---|---|
| **AMCL** | 粒子滤波 | 定位是**多峰**问题（走廊、对称房间）。EKF 的单高斯会崩（均值落在两个峰之间 = 不存在的位置）。粒子云天然支持多峰。 |
| **SLAM** | 图优化 | 如果 SLAM 用 EKF（EKF-SLAM），状态维度 = `位姿 + 所有地图点`，协方差是 **O(n²)**，地图一大就爆。图优化把问题稀疏化（一条边只连两个节点），才可扩展；而且支持重线性化 + 多次迭代。 |

**这是两个不同的问题结构，导致两种不同算法**，不是"谁更先进"。

##### 核心区别一览

| 维度 | **SLAM** | **AMCL** |
|---|---|---|
| **前提** | **不需要**地图 | **必须有**地图 |
| **输出** | 地图 **+** 位姿 | **只有**位姿 |
| **待估变量** | `(x_{1:t}, m)` 联合 | 只有 `x_t` |
| **地图的角色** | **待估变量** | **已知常量** |
| **主算法** | 扫描匹配 + 图优化 | 粒子滤波 |
| **复杂度来源** | 轨迹长度 × 地图规模 | 粒子数 × 光束数 |
| **累积误差体现在** | **地图上**（墙重影、变形）| 位姿上（但被观测持续拉回）|
| **典型失败** | 漂移、回环误判 | 绑架、环境退化 |
| **恢复手段** | 回环检测 + 图优化 | KLD + 随机注入 + `/initialpose` |
| **运行阶段** | 建图阶段（一次性/阶段性）| 运行阶段（每次开机）|
| **对应库** | `slam_toolbox` / `cartographer` | `nav2_amcl` |

##### 四个常见误区

| 误区 | 为什么错 |
|---|---|
| 「SLAM 也能定位，AMCL 多余」| AMCL 的价值在**重启后的快速稳定定位** —— 仓库机器人每天开机都要 |
| 「AMCL 能在没地图时用」| 不能。`map_server` 是硬依赖，没 `/map` 连初始化都做不了 |
| 「SLAM 建出来的图一定准」| 不一定。**回环只能修闭环误差，修不了尺度误差**（实测中间节点还差 10 cm）|
| 「两者可以同时跑」| 不能。都发 `map→odom`，见 §6.5 |

### 7. 局限与后续

#### 7.1 已知局限

诚实清单，按影响排序：

| # | 局限 | 影响 | 严重度 |
|---|---|---|---|
| 1 | **`PoseGraph` 没接进 `slam_mini_node`** | 只有前端，累积漂移完全没有后端消除；建长走廊会歪 | 🔴 高 |
| 2 | **回环检测没有扫描匹配验证** | 只按空间距离筛候选，相似走廊会误判成回环 → 地图撕裂 | 🔴 高 |
| 3 | **`/amcl_pose` 协方差写死** | 接不进两级 EKF（EKF 靠协方差决定信任度）；退化成功固定加权平均 | 🔴 高 |
| 4 | **距离场用 chamfer 近似** | 最大 8.24% 误差；打分够用，但**不能用于安全距离判断** | 🟡 中 |
| 5 | **beam model 缺 `z_short` / `z_max`** | 对动态障碍物、玻璃、镜面反射的鲁棒性差 | 🟡 中 |
| 6 | **没有 `update_min_d` / `update_min_a` 门控** | 静止时也在跑粒子滤波，浪费算力；且静止时位姿会缓慢抖动 | 🟡 中 |
| 7 | **单分辨率地图** | 大场地内存吃紧；扫描匹配没有多分辨率加速 | 🟡 中 |
| 8 | **扫描匹配暴力搜索** | 686 候选 × 光束数，比 `slam_toolbox` 的分支定界慢约 10 倍 | 🟡 中 |
| 9 | **没有 `save_map` 服务** | 只能 Ctrl+C 时存盘；`kill -9` 会丢掉整个地图 | 🟡 中 |
| 10 | **没有 RViz 配置** | 只能看话题数字，看不到图；演示效果差一半 | 🟢 低 |
| 11 | **`ekf_node` 未单独实测** | 会和 `robot_localization` 的 `ekf_local` 抢 `odom→base_link` | 🟢 低 |
| 12 | **算法层无耗时基准** | 只知道"10 Hz 不丢帧"，没有单帧耗时的精确数字 | 🟢 低 |

#### 7.2 后续可做（按投入产出排序）

| 优先级 | 事项 | 预估 | 价值 | 依赖 |
|---|---|---|---|---|
| **P0** | **`PoseGraph` 接进 `slam_mini_node`** | 4 h | ⭐⭐⭐ | — |
| **P0** | **`PoseFilter` 加 `poseCovariance()`，节点用真实协方差** | 1 h | ⭐⭐⭐ | — |
| **P1** | 回环验证（用 `ScanMatcher` 确认候选） | 1 h | ⭐⭐⭐ | P0 |
| **P1** | RViz 配置 + `/initialpose` 联动 | 1 h | ⭐⭐⭐ | — |
| **P1** | 和 `slam_toolbox` 跑同一段 bag 对比 | 4 h | ⭐⭐⭐ | P0 |
| **P2** | 距离场换成精确 Voronoi 波前（照 AMCL 的 `map_cspace`） | 2 h | ⭐⭐ | — |
| **P2** | `save_map` 服务 | 0.5 h | ⭐⭐ | — |
| **P2** | `mode:=mapping / localize` 单节点两种模式 | 4 h | ⭐⭐ | P0 |
| **P3** | `update_min_d/a` 门控 | 0.5 h | ⭐ | — |
| **P3** | `z_short` / `z_max` 补全 beam model | 2 h | ⭐ | — |
| **P3** | 单帧耗时基准 | 1 h | ⭐ | — |
| **P3** | 分支定界加速扫描匹配 | 8 h | ⭐⭐ | — |

**P0 的两项是"补齐核心能力"，其余是"打磨"。**

#### 7.3 这个包适合用来做什么

| 适合 | 为什么 |
|---|---|
| **学习 SLAM / AMCL 的原理** | 每个公式都能对上代码，37 个单元测试可交互验证 |
| **面试时讲清楚"我实现过什么"** | 有数据、有坑、有边界，不是"跑通了 demo" |
| **验证新的想法** | 算法层零 ROS 依赖，改一行 `colcon test` 3 秒出结果 |
| **教学/分享** | 代码量小（2500 行），能在一节课内讲完主干 |
| **离线处理 bag** | `map_io` 可直接读写，不需要起 ROS |

| 不适合 | 为什么 |
|---|---|
| **生产环境** | 比 `slam_toolbox` 慢 10 倍，且没有后端优化 |
| **大场地建图** | 单分辨率 + 暴力搜索，内存和算力都会顶不住 |
| **动态环境 / 人多场景** | beam model 缺 `z_short`，对动态障碍物鲁棒性差 |
| **需要安全距离判断的场合** | 距离场是 chamfer 近似，8% 误差不能用于避障 |
| **多机器人协同** | 没有命名空间隔离设计 |
| **3D / 多楼层** | 二维单层 |

#### 7.4 结语

这个包写了约 2500 行功能代码 + 1070 行测试，覆盖了 SLAM 和 AMCL 的核心链路。

它做不到的事情很多（见 7.1），但它做到了一件事：

> **每一行都能解释，每个参数都能说出所以然，每个失败都能定位到具体公式。**

用现成的库能做出功能；自己写一遍，才能在面试官问
「为什么这里要用粒子滤波而不是 EKF」「`recovery_alpha_slow` 在比较什么」
「回环为什么修不了尺度误差」的时候，给出一个有底气的答案。

**如果只记三句话：**

1. **SLAM 求 `P(轨迹, 地图 | 观测)`，AMCL 求 `P(位姿 | 观测, 地图)`** ——
   地图从"待估变量"变成"已知常量"，这一个差别派生出其余所有差别。

2. **似然模型不是越尖锐越好** —— 缺了 `z_rand` 这一项，粒子会瞬灭，
   一旦丢失就永远回不来。

3. **回环只能修闭环误差，修不了尺度误差** —— 尺度必须靠激光重新观测，
   或者多条互相约束的回环。
