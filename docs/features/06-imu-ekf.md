# IMU 与 EKF 传感器融合

> 本文是 README「已实现功能」的拆分子集，收录 **§25 ~ §25**（1 个小节）。
> 原始完整文档： [`docs/archive/README-2026-11-01-full.md`](../archive/README-2026-11-01-full.md) 第 753~885 行。

### 25. IMU 接入与 EKF 传感器融合


#### 传感器本身

URDF 里加 `imu_link`（0.03³ 方块、0.02 kg）+ `<sensor type="imu">` + `libgazebo_ros_imu_sensor.so`，
`update_rate 100`，陀螺 σ=0.0002 rad/s、加速度计 σ=0.017 m/s²，话题 `/imu`。

静止实测：`orientation.w ≈ 1`、`angular_velocity ≈ 0`、**`linear_acceleration.z ≈ 9.83`**。

> **加速度计测的是「比力」而不是「运动加速度」** —— 静止时地面支撑力向上 9.81 m/s²，
> 所以 z 分量是 **+9.81**。这个值接近 9.81 恰恰说明 IMU 正常；等于 0 反而是错的。

#### EKF 配置的三个关键选择

| 选择 | 原因 |
|---|---|
| **`two_d_mode: true`** | 差速车只有 x、y、yaw 三个自由度；还避免 IMU 微小俯仰噪声污染位姿 |
| **`publish_tf: false`** | `odom → base_footprint` 已由差速插件发布，EKF 再发一次会 TF 打架 |
| **不用 IMU 的 `orientation`** | 实测 `orientation_covariance` 全是 **0**（等于宣称"绝对精确"），会让滤波器无视其他所有输入；真实 IMU 的绝对朝向也易受干扰 |
| **不用 odom 的绝对 `yaw`** | 每 20 ms 一次的绝对朝向测量会压制角速度积分，让 IMU 的速率完全失效 |

协方差对比（决定融合权重）：

| 来源 | 角速度方差 | σ | 相对权重 |
|---|---|---|---|
| `/odom` 的 `twist.covariance` | 1e-3 | 0.032 rad/s | 1 |
| `/imu` 的 `angular_velocity_covariance` | 4e-8 | 0.0002 rad/s | **25000** |

#### 实验：原地转 13 秒，比较三种「转角」（干净数据）

| 数据源 | 总转角 | 相对真实误差 |
|---|---|---|
| **IMU 角速度积分（真实）** | **310.3°** | — |
| `/odom` 的 yaw | 307.6° | −2.7°（−0.9%） |
| `/odometry/filtered` | 306.9° | −3.3°（−1.1%） |
| 融合 − odom | −0.6° | 几乎无差别 |

#### 基线实验（不注入误差）：融合看不出收益

仿真中没有打滑 —— `/odom` 的角速度与 IMU **完全一致**（都是 0.4583 rad/s），
`/odom` 的 yaw 本身误差只有 0.9%。IMU 没有任何「可纠正的误差」可做，融合结果自然等于 odom。

**「测不出收益」本身是有效结论**：在理想仿真里讨论融合优化没有意义。
要真正验证融合价值，必须**人为注入误差源**：

| 方案 | 做法 | 预期 |
|---|---|---|
| 降低车轮摩擦 | 车轮加 `<mu1>0.05</mu1><mu2>0.05</mu2>` | 打滑 → odom 过报 → 融合纠正 |
| **伪造 odom 噪声** | rclpy 节点给 `/odom` 的 yaw 加 5% 漂移，另发 `/odom_noisy` | 误差量可控、可重复 |
| 给陀螺加零偏 | SDF 噪声 `<mean>` 设 0.02 rad/s | 看 odom 能否把漂移的 IMU 拉回来 |

#### 注入误差源：融合价值得到证明

基线实验说明「odom 准的时候融合看不出收益」，那就**人为把 odom 弄坏**，看融合能不能救回来。

**做法**：写 `odom_noise` 节点（rclpy），订阅 `/odom`，把角速度乘 1.10，
并用**污染后的角速度重新积分 yaw**（保证 `twist` 与 `pose` 自洽），发到 `/odom_noisy`；
再把 EKF 的 `odom0` 换成 `/odom_noisy`（`ekf_noisy.yaml`，其余配置完全不变）。

```
Gazebo /odom（准） ─┬──────────────────────────────────────┐
                    └─▶ [odom_noise ×1.10] ─▶ /odom_noisy ─▶[EKF]─▶ /odometry/filtered
Gazebo /imu（准）  ─────────────────────────────────────────▶[EKF]
```

**结果**（原地转，积分用 `header.stamp` 的仿真时间）：

| 数据源 | 总转角 | 相对真实误差 |
|---|---|---|
| **IMU 积分（真实）** | **224.9°** | — |
| `/odom`（未污染，作参照） | 224.9° | +0.0°（0.0%） |
| **`/odom_noisy`（注入 10%）** | **247.5°** | **+22.6°（+10.0%）** |
| **`/odometry/filtered`** | **222.9°** | **−2.0°（−0.9%）** |

**偏差消除 91%，精度提升 11 倍。** EKF 只看到「打了 10% 折扣的假轮速」和「真实的 IMU」，
却输出了误差 0.9% 的角度。时间线上误差**不随时间累积** ——
轮式里程计的偏差会一路累加，IMU 的角速度不会，这正是融合的价值所在。

**残余 0.9% 的解读**：按协方差之比（25000 : 1），融合结果本应几乎完全等于 IMU。
实测说明 EKF 实际仍给了 odom 约 9% 的权重 —— 这是**过程噪声矩阵与两个传感器采样率
（100 Hz vs 50 Hz）耦合**的结果。「协方差之比」只是权重的量级估计，不是精确公式。

> **涉及文件**：`src/my_robot_tasks/my_robot_tasks/odom_noise.py`、
> `src/my_robot_description/config/ekf_noisy.yaml`

#### 排查实录：八个把结论带偏的坑

这一节花了十几轮才定位，**过程比结论更有价值**。

| # | 坑 | 现象 | 正确做法 |
|---|---|---|---|
| 1 | **改 `src/` 不等于改 `install/`** | 改了 `ekf.yaml` 但行为不变 | `ros2 pkg prefix` 取到的是 install 副本；改完 `colcon build`，或直接用 src 路径启动 |
| 2 | **参数只在节点启动时读一次** | 编译后仍无效 | 必须重启节点，并用 `ros2 param get` 验证运行时的实际值 |
| 3 | **重复启动产生同名节点** | 两轮分析结论互相矛盾 | `ros2 node list` 出现两行同名节点、`/odometry/filtered` 的 `Publisher count: 2` → 数据被两个实例混合 |
| 4 | **丢帧让积分虚高** | `/odom` 的 `∫ωdt` 与自身 `yaw` 差 12° | 积分前先验 Hz（应 50/100/10）；`trapz` 遇丢帧会按匀速补，必须跳过间隔过大的段 |
| 5 | **不做自洽性检查** | 直接采信了不可能的数据 | 同一消息内 `∫twist.ω dt` 应等于 `Δpose.yaw`；对不上说明数据或算法有问题 |
| 6 | **把「指令 vs 实际」当成「轮式 vs 真值」** | 误判「有 8% 打滑」 | 指令 0.5 → 实际 0.4583 是**指令跟踪误差**；判断里程计漂移要比 `/odom` 与 IMU，而两者一致 |
| 7 | **时间轴运算用了 bag 记录时间** | `∫ωdt` 与 `Δyaw` 差 12°、5.4°，反复怀疑数据 | 必须用 **`header.stamp`（仿真时间）**。记录时间受主机负载影响、与仿真时间不同步；改用仿真时间后自洽性立刻恢复到 ±0.2° |
| 8 | **文件名与 `entry_points` 不一致** | `ros2 run` 报 `ModuleNotFoundError: my_robot_tasks.odom_noise` | 文件存成了 `dom_noise.py`（少一个 o）。`entry_points` 冒号右边是 `包名.文件名:函数名`，必须与磁盘文件名**完全一致** |
| **`ros2 run` 起来了但什么也不干** | `collision_monitor` 无订阅、无发布，像"没工作" | 它是 **Lifecycle 节点**，日志里写着 `Waiting on external lifecycle transitions to activate`。必须 `ros2 lifecycle set ... configure/activate`，或交给 `lifecycle_manager` |
| **矩形多边形的角落比正前方更早触发** | 前向 ±30° 最近距离 0.417 m，但 0.28 m 的急停区已经触发 | 矩形四角 `(0.28, ±0.22)` 距车心 0.356 m，斜前方 40~45° 的点先落进去。统计"最近距离"时要考虑角度 |
| **录 bag 前没确认目录创建** | 做完整套实验，最后发现没有任何新 bag | `ros2 bag record` 后必须看到 `Recording...`，并另开终端 `ls -ld bags/xxx` 确认目录存在、结束后 `du -sh` 确认不是 0 |
| **`status: 4` 是成功，不是放弃** | 把 `4` 当 ABORTED，白排查一整轮 | `STATUS_SUCCEEDED = 4`、`STATUS_CANCELED = 5`、`STATUS_ABORTED = 6`。**先查枚举再解读现象** |
| **`isTaskComplete()` 会在回调里二次 spin** | 定时器里调它，结果检查永远不触发 | 它内部调 `spin_until_future_complete`。在回调/定时器里改用 `self.result_future.done()` |
| **视觉目标必须落在视野锥内且无遮挡** | 检测器一条不发，任务节点干等 | 相机水平 FOV 60° → 只有 ±30°。放目标前用 `/scan` 确认正前方最近距离就是它 |
| **用被测函数算期望值 → 测试永远绿** | 测试全过，但函数返回 `(999,999)` 也不会红 | 期望值必须来自被测代码之外：手算、数学约束、几何常识、解析解、往返自洽 |
| **拿集成测试当单元测试用** | 60 秒一次，红了只知道"车没动" | 把数学抽成纯函数，0.14 秒筛一遍；集成测试只负责验证"接线" |
| **`pkill -f` 匹配到自己的命令行** | `bash -c 'pkill -9 -f gzserver; ...'` **把自己 -9 掉**，`exit code -9` | 模式首字母加方括号 `[g]zserver`；或改用 `pkill -x 进程名`（不匹配命令行） |
| **`pkill` 自匹配的根源是整条命令行** | 参数改对了还自杀，因为 `echo` 的提示语里写了 `gzclient` | 任何位置出现进程名都算。用 `-x` 或让提示语不含进程名 |
| **`;` 掩盖前一条命令的失败** | `pkill` 因畸形方括号失败，`echo` 照跑，日志显示"已关闭"但进程还在 | 用 `&&` / `||`；日志语句不能无条件执行 |
| **launch 是异步的** | `cleanup` 还没 `sleep 2` 完，`gazebo` 已经开始起 | `ExecuteProcess` 不等结束，必须用 `TimerAction` 串行化 |
| **`ros2 node list` 空但 `topic hz` 正常** | `lifecycle get` 报 `Node not found`，实际节点都在 | 数据面直连、元数据面走 daemon。`ros2 daemon stop && start` 或加 `--no-daemon` |
| **`/clock` 频率把全系统定时器压制** | EKF 配 30 Hz 只跑 9.5 Hz，改 `frequency` 无效 | `/gazebo` 的 `publish_rate` 默认只有 10 Hz。改 100 Hz 后 EKF 立刻跑到 30.06 Hz |
| **`/clock` 订阅要 best_effort** | 默认 reliable 订阅报 incompatible QoS，一条收不到，误以为没时钟 | `/clock` 是 BEST_EFFORT + VOLATILE，测量时要显式指定 |
| **`use_composition: False` 的代价** | 13 个 DDS participant，上下文切换 2 万次/秒，`sy` 45% | 改 `True` 后 participant 13→4，`sy` 45%→15% |
| **Fast DDS 共享内存抢端口** | `RTPS_TRANSPORT_SHM: fastrtps_port74xx open_and_lock_file failed`，gzserver 的 ROS 插件初始化失败 | `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 彻底禁用 SHM，报错消失 |
| **测量脚本自己会丢消息** | 用 `spin_once` 逐条处理高频话题，测出 8.4 Hz 而非真实值 | 用 `threading` + `rclpy.spin` 回调计数；或改用 `ros2 topic hz` |
| **`pgrep -c` 计数为 0 时 `\|\| echo 0` 会输出两个 0** | shell 判断 `[ "$n" -eq 0 ]` 报 `integer expression expected` | 用 `n=$(pgrep -c x); n=${n:-0}`，不用 `\|\|` |
| **`pkill -x` 对超过 15 字符的进程名无效** | `pkill -9 -x robot_state_publisher` 静默不杀任何进程，孤儿越积越多 | Linux `comm` 最多 15 字符（`robot_state_pub`）。**长名字一律用 `pkill -f "[r]obot_state_publisher"`** |
| **`pkill -9` 强杀 launch 会留下孤儿进程** | 反复重启后 DDS 图里堆了 7 个 `robot_state_publisher`，导致「Nav2 节点全部查不到」的假象 | 强杀 launch 不触发它优雅清理子进程。**用 SIGINT（Ctrl+C），或事后按进程名补杀** |
| **新进程 DDS 发现要 8~10 秒** | 启动 3 秒就查图，看到「`odom` 帧不存在」「节点为空」，误判系统坏了 | 查询前先等发现：轮询 `get_topic_names_and_types()` 直到看到关键话题 |
| **`ekf_global` 启动时没有绝对位姿** | `map→odom` 偏出 1.89 m，全局代价地图全偏 | `/amcl_pose` 受 `update_min_d/a` 门控，车不动就不发。**启动前用 `/request_nomotion_update` 强制 AMCL 更新一次** |
| **`collision_monitor` 的 STOP 是全向锁死** | 车撞到障碍后，**倒车也被拦**，彻底锁死只能重启 | `PolygonStop` 命中时把整条 `cmd_vel` 归零，不分方向。**实操上撞墙后不能靠倒车自救** |
| **`grep` 对带 ANSI 颜色的日志报 "binary file matches"** | 看不到匹配内容 | 加 `-a`，或先 `sed 's/\x1b\[[0-9;]*m//g'` 去色 |
| **`use_composition: True` 让容器内节点的 DDS 实体对外不可见** | `/amcl_pose` publisher=0，`ekf_global` 收不到，`map→odom` 纯推算漂移 | 容器内节点与外部进程通信时，先查 `get_publishers_info_by_topic`。**必要时关掉合成**（功能正确性优先于 CPU） |
| **"节点 active" 不等于"它的输出能到下游"** | `amcl` lifecycle active、日志正常，但 publisher 数为 0 | 用 `get_publishers_info_by_topic()` / `get_subscriptions_info_by_topic()` 查**实体级**可见性，不只看 lifecycle |
| **`robot_localization` 不会自动平滑 `pose0` 的硬跳变** | 以为两级 EKF 能吸收 AMCL 跳变，实测单步照样跳 0.35 m | 默认协方差下它把 `pose0` 当绝对量测直接吸收。要平滑需调大 `pose0` 协方差或用 `pose0_differential` |

> **可信判据 = 数据体检三条**：三个话题 Hz 符合预期（50 / 100 / 10）、
> `Publisher count` 为 1、自洽性误差在几度以内。三条都满足，结论才可信。

> **涉及文件**：`src/my_robot_description/urdf/robot.urdf`、`src/my_robot_description/config/ekf.yaml`
