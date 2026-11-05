# TF2 坐标变换

> 对应代码：`urdf/diff_robot.urdf.xacro`（静态 TF）、`config/ekf_local.yaml` / `ekf_global.yaml`（动态 TF）

## 1. 树的三条规则

1. 每个 frame 只能有一个父节点；
2. 每条边在同一时刻只能有一个发布者；
3. 任意两帧之间必须存在路径，否则查询报 `frame does not exist`。

本项目的树：

```
map ──(ekf_global)──▶ odom ──(ekf_local)──▶ base_link ──(static)──▶ laser
                                                  └──(static)──▶ imu_link
```

## 2. 静态与动态

| | 静态 | 动态 |
|---|---|---|
| 话题 | `/tf_static` | `/tf` |
| QoS | `TRANSIENT_LOCAL` + `RELIABLE` | `VOLATILE` |
| 发布者 | `robot_state_publisher` | `ekf_local` / `ekf_global` |
| 为什么这样 | 只发一次，晚加入者也要能拿到最后一条 | 高频更新，不需要历史 |

## 3. 时间戳是主要故障源

| 现象 | 原因 |
|---|---|
| `Transform data too old` | 缓存太旧：`transform_tolerance` 太小或发布频率不足 |
| `Lookup would require extrapolation into the future` | 查询时间比 buffer 最新还新 |
| 仿真里 `tf2_echo` 报帧不存在 | 未加 `--ros-args -p use_sim_time:=true`，用墙钟查仿真时间 |
| 回放 bag 时 TF 乱 | `ros2 bag play -l` 循环回放导致时间倒退，tf2 拒绝 |

排查顺序：**先看时间基准，再看帧名，最后看发布者**。

## 4. 两个发布者发同一条边

发布端不会报错，`sendTransform` 照样成功；订阅端建树时同一 child 出现两个 parent，只能保留一个，查询结果随机跳。

现象：RViz 位姿跳、costmap 抖、控制指令抖。
定位：`ros2 topic info /tf -v` 看 Publisher count；`view_frames` 看树里是否有重复边。

因此本项目关闭了 `amcl.tf_broadcast` 与 `diff_drive_controller.enable_odom_tf`，保证每段 TF 唯一发布者。

## 5. 四元数

- 顺序 `x, y, z, w`，`w` 为标量；
- 表示旋转的前提是模长为 1，不归一化会使旋转矩阵不正交、插值失真、姿态缓慢漂移；
- 欧拉角存在万向锁，内部统一用四元数。

## 6. base_link 与 base_footprint

- `base_link`：车体几何中心，含俯仰与滚转；
- `base_footprint`：`base_link` 在地面的投影（z = 0），只有 yaw。

2D 导航（costmap、AMCL）应使用 `base_footprint`，避免车体俯仰/悬挂变化引入平面误差。本项目因差速小车贴地运行，简化为 `base_link`。
