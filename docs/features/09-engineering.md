# 工程化：单元测试 / 一键启动 / 仿真时钟

> 本文是 README「已实现功能」的拆分子集，收录 **§28 ~ §30**（3 个小节）。
> 原始完整文档： [`docs/archive/README-2026-11-01-full.md`](../archive/README-2026-11-01-full.md) 第 1255~1689 行。

### 28. 单元测试：把设计契约定成断言


#### 起点：原来的代码根本没法测

`red_target_detector.py` 里的投影函数长这样：

```python
def project_to_ground(self, u, v, stamp):
    tf = self.tf_buffer.lookup_transform(...)   # 需要 ROS 节点 + TF
    self.get_logger().warn(...)                 # 需要 ROS 日志
```

**纯几何计算和 ROS 基础设施耦合** —— 想测「像素 → 地面」就得起 Gazebo、起 Nav2、等 TF。
那不是单元测试，是集成测试（60~90 秒一次，出错还定位不了）。

#### 第一步：可测性驱动设计

把数学从节点里拆出来，得到 `geometry.py`（**零 ROS 依赖**）：

```python
R_OPTICAL_TO_LINK                                    # 常量矩阵
quat_xyzw_to_matrix(x, y, z, w)      -> 3x3           # 收 4 个 float，不收 ROS 对象
pixel_to_ray(u, v, K)                -> 单位向量
optical_to_world(d_opt, R)           -> 世界系方向
intersect_ground(origin, direction)  -> (x, y) | None
```

**每个函数就是一个「契约」。测试 = 把契约定成断言。**

> `quat_xyzw_to_matrix` 故意收 4 个 float 而不是 ROS 的 `Quaternion` 对象 —— 就是为了能在测试里随手造数据。

#### 第二步：对每个函数问三个问题

| 问题 | 对应的测试类型 |
|---|---|
| **它承诺了什么？** | 正常路径测试 |
| **有没有「不跑代码就知道答案」的例子？** | 已知答案测试 |
| **哪些输入必须被拒绝？** | 异常路径测试 |

按这个框架填满 13 个测试：

| 被测函数 | 正常路径 | 已知答案 | 异常路径 |
|---|---|---|---|
| `R_OPTICAL_TO_LINK` | 三个轴的映射 | 手算 `(0,-1,0)` / `(0,0,-1)` / `(1,0,0)` | **必须是真旋转**（正交 + det=+1） |
| `quat_xyzw_to_matrix` | 单位元 | `(0,0,0,1)` → 恒等矩阵 | yaw=90° → 解析解 |
| `pixel_to_ray` | 主点 → `(0,0,1)` | 模长必须为 1 | 右侧像素 → x>0 |
| `intersect_ground` | 正下方 → 原点 | 45° → 水平位移 = 相机高度 | **朝上 / 平行 → None** |

其中这条断言直接对应 §27 踩过的 bug：

```python
def test_intersect_upward_returns_none():
    assert intersect_ground([0, 0, 0.08], [0, 0, 1]) is None
    assert intersect_ground([0, 0, 0.08], [0.5, 0.0, 0.5]) is None
```

> 投箱子重心时射线朝上，与地面永不相交 → 静默返回 None。**以后谁改坏了，这条立刻变红。**

#### 第三步（灵魂）：期望值必须来自被测代码之外

**错误做法** —— 用被测函数算期望值：

```python
expected = intersect_ground([0, 0, 0.08], [0, 0, -1])          # ← 用答案验证答案
assert intersect_ground([0, 0, 0.08], [0, 0, -1]) == expected
```

**这个测试永远绿，哪怕函数返回 `(999, 999)`。**

本项目用了 **5 种独立的期望值来源**：

| 来源 | 例子 | 为什么可信 |
|---|---|---|
| **① 手算** | `R_OPTICAL_TO_LINK @ (1,0,0) == (0,-1,0)` | 图像向右 → 车体 −Y，这是坐标定义 |
| **② 数学/物理约束** | `‖d‖ == 1`、`det(R) == +1`、`R·Rᵀ == I` | 旋转矩阵的定义，与实现无关 |
| **③ 几何常识** | 45° 斜射 → 水平位移 = 相机高度 | 等腰直角三角形 |
| **④ 教科书解析解** | 主点 `(cx,cy)` → 射线 `(0,0,1)` | 针孔相机模型的定义 |
| **⑤ 双向自洽（roundtrip）** | 地面点 → 像素 → 地面点 | **不需要知道中间答案** |

#### 第五种最巧妙：往返测试

```python
target = (1.0, -0.5)                       # 已知地面点
# 反算：地面点 → 像素 → 正投影回地面
assert p == pytest.approx((1.0, -0.5), abs=1e-9)
```

**只需要一个约束：正向变换和反向变换必须互为逆运算。**

这是 **oracle problem 的经典解法**：当你不知道正确答案时，去找一个不需要正确答案的约束。

一次往返就能同时抓住：`K` 用错、光学旋转矩阵写错、四元数公式错、漏归一化、求交公式错 ——
**任何一环错，往返都不闭合。** 它是 13 个测试里最「贵」的一个（也包含了一个 yaw=30° 的版本）。

#### 集成测试 vs 单元测试的分工

| | 集成测试（跑仿真） | 单元测试（pytest） |
|---|---|---|
| 耗时 | 60~90 秒 | **0.14 秒** |
| 需要什么 | Gazebo + Nav2 + 红箱子 + 正确视角 | 只要 Python |
| 出错能定位吗 | ❌「车没动」—— 20 个可能原因 | ✅ 直接指出是哪个函数 |
| 能造边界输入吗 | ❌ 很难造「射线朝上」 | ✅ 一行代码 |

```
单元测试  →  保证「数学是对的」     ← 错了整个系统必错
集成测试  →  保证「接线是对的」     ← 跑一次就能看出来
```

#### 验证测试真的有效：变异测试（故意改坏）

只看「通过」不会让人相信测试，**只有见过它变红才会**：

| 故意改坏的地方 | 应该变红的测试 |
|---|---|
| `R_OPTICAL_TO_LINK` 加个 `.T`（转置） | `test_optical_to_link_axes`、`test_full_roundtrip`、`test_roundtrip_with_yaw` |
| `pixel_to_ray` 删掉 `/ norm(d)` | `test_pixel_ray_is_unit_length` |
| `intersect_ground` 注释掉 `direction[2] >= 0 → None` | `test_intersect_upward_returns_none` |

#### 可测代码的四个特征

| 特征 | 反例 | 正例 |
|---|---|---|
| **纯函数**（同样输入永远同样输出） | 读全局变量、读时钟 | 参数传进来 |
| **无副作用** | 函数里发 ROS 消息 | 只 `return` |
| **依赖注入** | 内部自己建节点 | 需要的东西当参数传 |
| **单一职责** | 一个函数既查 TF 又算数学又发消息 | 拆三个函数 |

`geometry.py` 四条全满足，所以能 0.14 秒测完。

> **涉及文件**：`src/my_robot_tasks/my_robot_tasks/geometry.py`、
> `src/my_robot_tasks/test/test_projection.py`

### 29. 一键启动：launch 参数化与编排


#### 痛点

之前每次实验要开 **7 个终端、敲 12 条命令、等 3 次"凭感觉"的时长**：

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f nav2
pkill -9 -f collision_monitor; pkill -9 -f robot_state_publisher
ros2 daemon stop; sleep 3
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
export DISPLAY=:0 && ros2 launch my_robot_description gazebo.launch.py
# 等 15 秒；pkill -f gzclient
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/ros2/maps/my_map.yaml \
  params_file:=$HOME/ros2/src/my_robot_description/config/nav2_params.yaml \
  use_sim_time:=true use_composition:=False
# 等 Managed nodes are active；再等 10 秒
ros2 service call /set_initial_pose nav2_msgs/srv/SetInitialPose ...
```

换台机器、换个目录、隔一天回来 —— 全都要重来，**漏一条就出怪问题**（`/odom` 消失、Nav2 心跳超时全栈停用）。

#### 目标：一条命令

```bash
ros2 launch my_robot_description bringup_all.launch.py
```

#### 四个阶段

| 阶段 | 动作 | 时机 | 实现 |
|---|---|---|---|
| 0 | 清残留进程 + 清 `/dev/shm/fastrtps_*` | 立即 | `ExecuteProcess` |
| 1 | Gazebo（含 Gazebo 的 `gazebo.launch.py`） | +4 s | `IncludeLaunchDescription` + `TimerAction` |
| 2 | Nav2（含 `nav2_bringup` 的 `bringup_launch.py`） | +12 s | 同上 |
| 3 | 杀 `gzclient` 省 CPU（可关） | +15 s | `TimerAction` + `IfCondition` |

#### 三个核心机制

**① `ExecuteProcess` —— launch 里能跑任意 shell 命令**

```python
ExecuteProcess(cmd=['bash', '-c', 'pkill ...; sleep 2; rm -f /dev/shm/fastrtps_*'], output='screen')
```

`cmd` 是**列表**；`['bash','-c','xxx']` 才能用 `;`、`&&`、`$()`。`output='screen'` 把输出打到终端。

**② ⚠️ launch 是异步的 —— 这是第一个大坑**

```python
return LaunchDescription([
    cleanup,     # ← 不会等它跑完
    gazebo,      # ← 立刻开始起
])
```

`ExecuteProcess` 启动进程就返回，不等结束。必须用 `TimerAction` 串行化：

```python
TimerAction(period=4.0,  actions=[gazebo]),   # 等清理跑完
TimerAction(period=12.0, actions=[nav2]),     # 等 Gazebo spawn 完车、/scan 有数据
```

> **手动操作时"感觉没事"，是因为敲命令本身是串行的。**
> 从"手动"到"脚本"最容易忽略这个差异。

**③ `PathJoinSubstitution` + `FindPackageShare` —— 不写死路径**

```python
map_yaml = PathJoinSubstitution([
    FindPackageShare('my_robot_description'), 'maps', 'my_map.yaml'])
nav2_params = PathJoinSubstitution([
    FindPackageShare('my_robot_description'), 'config', 'nav2_params.yaml'])

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(...bringup_launch.py),
    launch_arguments={'map': map_yaml, 'params_file': nav2_params,
                      'use_sim_time': 'true', 'use_composition': 'False'}.items(),
)
```

`launch_arguments` 等价于命令行的 `key:=value`；`.items()` 不能漏。

**前置条件**：地图必须搬进包里（见 §18），否则 `FindPackageShare` 找不到。
搬迁用 `git mv`，git 显示 `rename {maps => ...}/my_map.pgm (100%)`，历史完整保留。

#### 实测时间线

| 时刻（launch 后） | 事件 |
|---|---|
| ~1 s | `[bringup] 清理完成，残留 fastrtps 段: 0` |
| ~5 s | Gazebo 起来、`Successfully spawned entity [my_car]` |
| ~7 s | `diff_drive` 订阅 `/cmd_vel_final` |
| ~12 s | Nav2 十个进程启动 |
| ~15 s | `[bringup] 已关闭 Gazebo GUI（headless）` |
| ~15 s | `map_server` / `amcl` 配置完成，地图加载 110×81 @ 0.05 |
| ~15.6 s | `lifecycle_manager_localization: Managed nodes are active` |
| ~18 s | `lifecycle_manager_safety` / `lifecycle_manager_navigation` 全部 active |

**18 秒 → 全栈就绪。**

> 顺带收获：12 秒延时让 AMCL 激活时 `/scan` 与 `/map` 都已就绪，
> `set_initial_pose: true` 的自动初始化**这次真的生效了**（日志里 `Setting pose (...): 0.000 0.000 0.000`）。

#### 排查实录：四个坑

**① `pkill -f` 会匹配到自己的命令行 —— 两次**

```python
cmd = ['bash', '-c', 'pkill -9 -f gzserver; pkill -9 -f nav2; ...']
#                             ↑ 这条 bash 的命令行里就有 "gzserver"
```

`pkill -f` 匹配**完整命令行**，所以它找到了自己，**把自己 -9 掉**：

```
[ERROR] [bash-1]: process has died [pid 3217, exit code -9, cmd 'bash -c pkill -9 -f gzserver; ...']
```

**修法：模式首字母加方括号**（`[g]zserver`），正则仍匹配 `gzserver`，但自己的命令行里是 `[g]zserver`，不含连续子串 → 不匹配自己。

**第二次踩得更隐蔽**：`pkill` 参数改对了，但 **`echo` 的提示文字里写了 `gzclient`**：

```bash
bash -c pkill -f "[g]zclient" && echo "[bringup] 已关闭 gzclient（headless 模式）"
#                                            ↑ 这里有字面量 gzclient，照样自匹配
```

**根因是"整条命令行里任何地方出现那个进程名都算"。** 两种彻底修法：

| 修法 | 说明 |
|---|---|
| **`pkill -x 名字`** | `-x` 匹配**进程名**而非命令行，`bash` 永远不会匹配 `gzclient` |
| 提示语不写进程名 | 改成"已关闭 Gazebo GUI" |

**能用 `-x` 的前提**：目标进程的进程名就是它自己。`gzserver` / `gzclient` / `collision_monitor` / `robot_state_publisher` 都可以；
但 `ros2 run` / `ros2 launch` 起的 Python 节点进程名都是 `python3`，**只能用 `-f` + 方括号**。

**② `;` 会掩盖前面的失败 —— 日志说假话**

```bash
pkill -f [gzclient; echo "[bringup] 已关闭 gzclient"   # [gzclient 是畸形方括号
```
`pkill` 报 `Unmatched [, [^, [:, [., or [=` 失败，但 **`;` 让后面的 echo 照跑** → 日志显示"已关闭"，
实测 `pgrep -c gzclient` 是 **1**。

**修法：`&&` / `||` 让日志说真话**

```bash
pkill -9 -x gzclient && echo "已关闭 Gazebo GUI" || echo "警告：没杀掉"
```

**③ `ros2 daemon` 缓存陈旧 —— `Node not found` 不可信**

```
ros2 node list          → 空
ros2 lifecycle get /amcl → Node not found
但 ros2 topic hz /odom   → 47.8 Hz 正常
```

**数据面（topic hz / echo）自己建节点直连，不经过 daemon；元数据面（node list / lifecycle get）要问 daemon。**
daemon 缓存过期就出现"进程活着但节点找不到"。

**修法**：`ros2 daemon stop && ros2 daemon start`，或给命令加 `--no-daemon`。

**④ 看 `status: 4` 别当成失败**

`STATUS_SUCCEEDED = 4`。这个坑详见 §27。

> **涉及文件**：`src/my_robot_description/launch/bringup_all.launch.py`、
> `src/my_robot_description/launch/gazebo.launch.py`、`src/my_robot_description/launch/gazebo.launch.py`

### 30. 仿真时钟陷阱：`/clock` 的 10 Hz 把全系统定时器压制了


#### 现象

两级 EKF 的第 1 级 `ekf_local` 配置了 `frequency: 30.0`，但实测输出只有 **9.5 Hz**。

```
$ ros2 topic hz /odometry/local
average rate: 9.462      # 配置是 30，只跑到 28%
```

而且这个数字**极其稳定**（8.3 / 8.4 / 9.4 / 9.5 / 9.6），不像 CPU 抖动那样杂乱。

#### 排查：一个个排除

| 假设 | 实验 | 结果 |
|---|---|---|
| **CPU 不够** | 只起 Gazebo + EKF，不起 Nav2（load 3.49，空闲 64%） | 还是 9.66 Hz ❌ |
| **GUI 抢 CPU** | 杀掉 `gzclient`（它占 70% CPU） | 8.33 → 9.44 Hz，只有 +13% ❌ |
| **相机渲染太贵** | 分辨率 640×480 降到 320×240，数据量 1/4 | 相机 2.84 → 1.82 Hz，EKF 没变 ❌ |
| **相机关掉** | `always_on=false, visualize=false` | EKF 还是 9.38 Hz ❌ |
| **`frequency` 没生效** | 分别设 10 / 30 / 60 | **全部 9.5 Hz** ❌ |
| **`sensor_timeout` 卡住** | 分别设 0.02 / 0.5 | 9.61 / 9.69 Hz ❌ |

`ros2 param get` 确认 `frequency` 是 30.0，参数确实加载了。

#### 根因：`/clock` 只有 10 Hz

关键推理链：

```
/odom 42 Hz 正常    ← Gazebo 插件按"物理步"驱动，与 ROS 时钟无关
/odometry/local 9.5 Hz ← ROS 定时器，use_sim_time=true
```

`use_sim_time:=true` 的节点，其 ROS 定时器**只在仿真时钟更新时才能触发**。
所以定时器的实际上限 = `/clock` 的发布频率。

实测（用 `best_effort` QoS 才能收到，`/clock` 是 BEST_EFFORT + VOLATILE）：

```
/clock: 9.5 Hz
仿真时间 3.80 s / 墙钟 4.00 s  →  RTF 0.949
```

**每条 `/clock` 覆盖 0.1 s 仿真时间** → 30 Hz 定时器（周期 33 ms）每 0.1 s 最多触发一次
→ **被压到 10 Hz**。

> **为什么改 `frequency` 到 10/30/60 都没用**：上限是时钟更新率，不是定时器配置。

#### 修复：`/gazebo` 的 `publish_rate` 参数

```bash
$ ros2 param list /gazebo
    publish_rate          ← 就是它，默认 10.0
    use_sim_time
    ...
```

`gzserver` 加载的 `libgazebo_ros_init.so` 用这个参数节流 `/clock`。

**新建 `config/gazebo_params.yaml`**：

```yaml
gazebo:
  ros__parameters:
    publish_rate: 100.0
```

**在 `gazebo.launch.py` 里把它传给 gzserver**：

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(...gazebo_ros/launch/gazebo.launch.py),
    launch_arguments={
        'world': world_path,
        'params_file': os.path.join(pkg_share, 'config', 'gazebo_params.yaml'),
    }.items(),
)
```

**结果**：

| 指标 | 改前 | 改后 |
|---|---|---|
| `publish_rate` | 10.0 | **100.0** |
| `/clock` | **9.5 Hz** | **77.6 Hz** |
| **`/odometry/local`** | **9.5 Hz** | **30.06 Hz** ✅ 正好达标 |
| `odom→base_footprint` TF | 9.5 Hz | **30.0 Hz** ✅ |

> `publish_rate` 不是越高越好：1000 Hz 会让每个节点把 CPU 全花在消化 `/clock` 上
> （这台虚拟机就是这样，实测只能处理 ~10 条/秒）。**100 Hz 是够 30 Hz 定时器用、
> 又不会压垮节点的甜点**。

#### 顺带四组降载措施（都实测有效）

| 措施 | 文件 | 效果 |
|---|---|---|
| **Nav2 组件化** `use_composition: True` | `bringup_all.launch.py` | DDS participant 13→4，上下文切换 −47%，`sy` 45%→15% |
| **禁用 Fast DDS 共享内存** `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` | 同上 | `RTPS_TRANSPORT_SHM Error: fastrtps_port74xx` **彻底消失**（这个报错会让 gzserver 的 ROS 插件初始化失败，`/odom` `/scan` 全没有） |
| **相机 640×480@15 → 320×240@5** + `visualize=false` | `robot.urdf` | 相机从跑不动变成 4.57 Hz |
| **物理 1000 Hz → 250 Hz**（步长 1 ms → 4 ms） | `worlds/my_world.world` | RTF 0.949 → 0.999，Gazebo CPU 大降 |

> 最后那项：`my_world.world` 原本**没有 `<physics>` 段**，用的是 Gazebo 默认
> `max_step_size=0.001 / real_time_update_rate=1000`。差分驱动车不需要 1 ms 精度。

#### 方法论：怎么判断"定时器被时钟压制"

遇到「ROS 定时器配置的频率达不到」，按这个顺序排除：

```
① 换 frequency 值，输出跟不跟？
   不跟 → 不是定时器配置问题
② 关掉一切负载再测，还不变？
   不变 → 不是算力问题
③ 量 /clock 的实际频率（注意 QoS 要用 best_effort）
   和定时器频率对比 —— 若 /clock 频率 ≤ 目标频率，就是它
④ 检查 clk 每条消息覆盖的仿真时间：仿真时间增量 / 消息数 > 1/frequency 即坐实
```

> **排查里最费时间的不是修，是排除。** 这一轮排掉了 CPU、GUI、相机、`frequency`、
> `sensor_timeout` 五个假设，每个都要一次完整重启验证。**所以先做"改参数就能验证"的实验
> （`frequency` 10/30/60），再做"要重启才能验证"的实验。**

> **涉及文件**：`src/my_robot_description/config/gazebo_params.yaml`、
> `src/my_robot_description/launch/gazebo.launch.py`、
> `src/my_robot_description/launch/bringup_all.launch.py`、
> `src/my_robot_description/worlds/my_world.world`、
> `src/my_robot_description/urdf/robot.urdf`
