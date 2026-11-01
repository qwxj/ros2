# 传感器仿真与 Launch 管理

> 本文是 README「已实现功能」的拆分子集，收录 **§13 ~ §18**（6 个小节）。
> 原始完整文档： [`docs/archive/README-2026-11-01-full.md`](../archive/README-2026-11-01-full.md) 第 272~367 行。

### 13. 速度指令看门狗（cmd_vel_watchdog）


- `/cmd_vel` 是**设定值**而非一次性命令：差速驱动插件会把最后收到的速度一直保持下去，
  遥控端掉线时机器人会一直冲（Gazebo 的差速插件本身**没有任何超时机制**）。
- 自写节点 `cmd_vel_watchdog`：把 `/cmd_vel` 转发到 `/cmd_vel_safe`；
  若 `timeout_sec` 内没有新指令，就主动发布**全零**（注意不是"停止发布"——停止发不会刹车）。
- 超时用 `std::chrono::steady_clock` 而不是 `this->now()`：Gazebo 一旦暂停，ROS 时间会冻住，
  用仿真时间会让看门狗**彻底失效且不报错**。
- 超时阈值必须**大于**指令发布周期（`τ ≥ 2~3 × T`），否则每个周期都会误报，
  机器人表现为"走一下、停一下"。
- `timeout_sec` 做成参数，可用 `ros2 param set` 运行时调整，直观体会"安全性 vs 可用性"的取舍。

- **架构演进**：最初看门狗是「旁路」形态（`/cmd_vel_input` → `/cmd_vel`），接入 Nav2 后与
  `velocity_smoother` 抢同一个话题，导致车一顿一顿；现改为**串联安全层**，详见 §19。

> **涉及文件**：`src/my_first_pkg/src/cmd_vel_watchdog.cpp`

### 14. 激光雷达与仿真环境


- 在车顶加 `lidar_link`（fixed 关节），用 `<sensor type="ray">` + `libgazebo_ros_ray_sensor.so`
  输出二维扫描；`output_type` 必须显式写 `sensor_msgs/LaserScan`（不写默认是 `PointCloud2`）。
- 通过 `~/out:=scan` 重映射把话题改成 `/scan`，用 `frame_name` 指定 `frame_id`。
- 扫描参数：360 线 × 整圈（±π）、量程 0.12~8 m，并叠加 σ=1 cm 高斯噪声以贴近真实雷达。
- 实测 `/scan` 约 9.9 Hz（配置 10 Hz）；`angle_increment` 为 0.0175 rad ——
  Gazebo 用的是 `(max-min)/(samples-1)`，与 ROS 惯例的 `/samples` 不同，读数要以消息字段为准。
- 空世界里雷达全是 `inf`（水平扫描打不到地面）；加入障碍物后测距值与几何尺寸吻合
  （墙在 x=-2.0 时读数约 1.94~2.07 m）。

> **涉及文件**：`src/my_robot_description/urdf/robot.urdf`、`src/my_robot_description/worlds/my_world.world`

### 15. 相机与 cv_bridge 图像处理


- 车头加 `camera_link`，用 `<sensor type="camera">` + `libgazebo_ros_camera.so`
  输出 640×480、`rgb8`、15 fps。
- 相机插件的话题命名规则是 `/<namespace>/<camera_name>/image_raw`，
  `namespace` 与 sensor 的 `name` 会**叠加** —— 两者都叫 `camera` 会得到 `/camera/camera/image_raw`，
  只保留 sensor name 才是 `/camera/image_raw`。
- 图像处理节点 `image_processor`：用 `cv_bridge::toCvCopy(msg, "bgr8")` 把 ROS 图像转成 `cv::Mat`
  （同时把 `rgb8` 转成 OpenCV 习惯的 BGR），再做灰度化 + Canny 边缘检测。
- 反向打包用 `cv_bridge::CvImage`：必须**继承原消息的 `header`**（时间戳 + frame_id），
  `encoding` 要设成 `mono8`（单通道）；发布时需 `publish(*out_msg.toImageMsg())` 解引用，
  因为 `Publisher::publish()` 没有接受 `shared_ptr` 的重载。
- 在 RViz2 用 `Image` 显示项验证：绿方块占 181×180 像素，Canny 边缘 bbox 与之完全重合，
  边缘像素 724 ≈ 矩形周长 2×(181+180)=722。

> **涉及文件**：`src/my_first_pkg/src/image_processor.cpp`、`src/my_first_pkg/config/image_processor.yaml`、
> `src/my_first_pkg/launch/image_processor.launch.py`

### 16. Launch 启动管理


- 使用 Python 编写 launch 文件。
- 实践多个节点的统一启动、参数传递及话题重映射。
- 通过 launch 文件组织多个节点的运行流程。

> **涉及文件**：`src/my_first_pkg/launch/demo.launch.py`（启动 `my_publisher`、`my_subscriber`、`parameter_demo`，加载 `parameter_demo.yaml`）、
> `src/my_first_pkg/launch/multi_node.launch.py`（启动 `my_publisher`、`status_node`，加载 `params.yaml`）、
> `src/my_robot_description/launch/display.launch.py`（启动 `robot_state_publisher` + RViz2）

### 17. 数据录制与离线回放（rosbag2）


- 用 `ros2 bag record` **显式列出话题**录制（不用 `-a`），产出两个数据集：
  `bags/drive_01`（4 话题 / 37 s / 7436 条 / 3.7 MiB）与
  `bags/with_cam`（`/camera/image_raw` + `/scan` / 19 s / 442 条 / 234 MiB）。
- 实测各话题数据量差异极大：`/scan` 9.5 Hz、`/odom` 47 Hz、
  **`/tf` 143 Hz（消息数比 `/scan` 大 15 倍）**、`/camera/image_raw` 14 Hz
  （单帧 640×480×3 ≈ 0.9 MB，10 Hz 即 9 MB/s，20 秒约 180 MB）。
  所以录制必须显式选话题 + 控制时长，盲目 `-a` 会让 bag 迅速膨胀。
- 回放用 `ros2 bag play --clock 1000`，由**播放器自己产生 `/clock`**；
  因此录制时**不需要录 `/clock`**（它 1000 Hz，录进去纯属浪费），
  而 `use_sim_time` 的节点在没有 Gazebo 时也能正常工作。
- 验证了**数据与算法解耦**：Ctrl+C 关掉 Gazebo 后回放 `bags/with_cam`，
  `image_processor` 照常处理并发布 `/image_processed`，RViz 正常显示边缘图 ——
  调算法不必再启动仿真、开车对准目标，且每次输入完全一致（可复现）。
- 常用参数：`-r 0.5` 慢放、`-l` 循环、`--topics` 只放部分话题、
  `-p` 启动即暂停、`--start-offset` 跳过开头、`--max-bag-duration` 限制录制时长。

> **涉及目录**：`bags/`（录制数据，已在 `.gitignore` 中排除，不入库）

### 18. SLAM 建图（slam_toolbox）


- 用 `slam_toolbox` 跑通「激光扫描 + 里程计 → 栅格地图」的完整链路：
  实时开车用 `online_async_launch.py`（异步、不阻塞），喂 bag 用 `online_sync_launch.py`。
- SLAM 在 TF 树里**只负责发布一段 `map → odom`**：通过扫描匹配估计机器人在 `map` 中的位姿，
  再把 `odom` 的累积漂移补偿掉。另外两段由别人提供 ——
  `odom → base_footprint`（差速插件）、`base_footprint → base_link → lidar_link`（robot_state_publisher）。
- 关键参数：`base_frame: base_footprint`、`scan_topic: /scan`、`resolution: 0.05`、
  `minimum_travel_distance / heading`（走多远、转多少插一个节点，仿真小车调到 0.2）、
  `map_update_interval`（地图刷新周期）、`min/max_laser_range`（与 URDF 量程一致）。
- 建图必须**真的移动**：原地不动只有一帧扫描，地图长不出来；
  绕一圈回到起点会触发**闭环检测（loop closure）**，能看到地图自动对齐修正。
- 成果 `src/my_robot_description/maps/my_map.pgm` + `my_map.yaml`：110×81 px @ 0.05 m/px = **5.5 m × 4.05 m**，
  空闲 76.9% / 障碍 2.4% / 未知 20.7%，`origin` 与 world 中障碍物位置吻合。

> **涉及文件**：`src/my_robot_description/config/slam_toolbox.yaml`、
> `src/my_robot_description/maps/my_map.pgm`、`src/my_robot_description/maps/my_map.yaml`
