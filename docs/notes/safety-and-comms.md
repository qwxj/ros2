# 安全链、QoS 与通信机制

> 对应代码：`cmd_vel_watchdog.cpp`、`cmd_vel_adapter.py`、`collision_monitor.yaml`、`qos_publisher.cpp`、`loaned_message_demo.cpp`

## 1. 速度指令链

```
/cmd_vel ──▶ watchdog ──▶ /cmd_vel_safe ──▶ collision_monitor ──▶ /cmd_vel_final
        ──▶ cmd_vel_adapter ──▶ /diff_drive_controller/cmd_vel_unstamped ──▶ 底盘
```

| 环节 | 实现 | 职责 |
|---|---|---|
| watchdog | C++ 节点 | 0.5 s 未收到指令则持续发布零速度 |
| collision_monitor | Nav2 节点 + yaml | 按多边形区域降速或急停；激光断流也停 |
| adapter | Python 节点 | 私有话题与标准话题之间的改名转发 |

watchdog 与 `diff_drive_controller.cmd_vel_timeout` 并存：后者只保护单个控制器，且控制器被切换或未激活时失效；watchdog 位于更上游，与控制器无关，且零速输出在话题上可见、可录包。

`collision_monitor` 是 LifecycleNode，未经 configure/activate 时整条链无输出，底盘不会运动。

## 2. QoS 三类数据

| 数据 | 选型 | 原因 |
|---|---|---|
| 传感器流（`/scan`、`/imu`） | `BEST_EFFORT` + `KEEP_LAST(1~5)` | 只关心最新帧，重传会造成延迟堆积 |
| 控制指令（`/cmd_vel`） | `RELIABLE` + `KEEP_LAST(1)` | 必须送达，但旧指令无意义 |
| 地图 / 静态（`/map`、`/tf_static`） | `RELIABLE` + `TRANSIENT_LOCAL` | 后加入的订阅者也要拿到已发布内容 |

两种不兼容现象：

- 直接不匹配：订阅者 `RELIABLE` 而发布者 `BEST_EFFORT`，连接建立失败，订阅端无数据；
- 兼容但无历史：订阅者 `VOLATILE` 订阅 `TRANSIENT_LOCAL` 发布者，只能收到之后的新消息（表现为 `/map` 在地图已发布后才启动的节点中为空）。

排查：`ros2 topic info <topic> -v`。

## 3. 执行器与回调组

单线程执行器中所有回调串行执行。一个耗时回调会占住执行器线程，到达期限的定时器无法被调度，且错过的 tick 不补发，表现为话题频率下降。

三种处理方式：

1. `MultiThreadedExecutor` + 回调组（`MutuallyExclusive` / `Reentrant`）；
2. 按快慢拆分回调组，隔离耗时任务；
3. 回调只入队，由独立线程消费（`worker_queue_demo`）。

`MutuallyExclusive` 组内仍然串行；`Reentrant` 组内可并发，需自行保证线程安全。

## 4. 零拷贝（loaned message）

普通发布需要序列化并拷贝；loaned 消息向中间件借用共享内存，直接写入后只传句柄。

必须使用定长消息类型的原因：

- 内存池按固定尺寸预分配，变长消息无法建立固定块；
- 变长类型（`uint8[]`、`string`）内部含指针，跨进程共享内存中指针失效；
- 定长数组内联存储，整个结构是一块连续内存，可按相同偏移映射。

`BigLoanedData` 使用 `uint8[1048576]`。启用条件：RMW 支持（`can_borrow`）且 Fast DDS 开启 `data_sharing`（见 `DEFAULT_FASTDDS_PROFILES.xml`）。

测量方法：消息携带序号与发送时间戳，订阅端计算 `now − stamp`；同机、1 MB、10 Hz 条件下对比普通发布与 loaned 发布，取多轮均值。本项目实测端到端延迟下降约 30%，剩余开销来自 DDS 唤醒、调度与队列。
