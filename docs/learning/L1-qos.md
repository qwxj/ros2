# L1 通信层（一）：QoS 不是"选个参数"，是"签个合同"

> **目标**：给定两个 QoS 配置，能判断它们**连不连得上**、连上后**数据怎么丢**。
> **前置**：L0（能独立 build 一个包）。
> **产出**：一张填满的兼容性矩阵表 + 能解释 `matched` 为什么是 0。

---

## 1. 七个策略，各自管什么

| 策略 | 常见取值 | 管什么 | 配错了的症状 |
|---|---|---|---|
| Reliability | `reliable` / `best_effort` | 丢了要不要重传 | 订阅端一条数据都收不到 |
| Durability | `volatile` / `transient_local` | 晚加入的订阅者要不要补历史 | 同上，且 `ros2 topic echo` 也没数据 |
| History | `keep_last(n)` / `keep_all` | 本地缓存留几条 | 高频时丢包、内存涨 |
| Depth | 整数 | keep_last 的 n | 慢订阅者被覆盖，`lost` 增长 |
| Deadline | 周期 | "我承诺最多隔多久发一次" | 违约告警/事件 |
| Lifespan | 时长 | "这条消息多久后作废" | 晚加入拿到过期数据（如果没设） |
| Liveliness | `automatic` / `manual_by_topic` + lease | 怎么证明"我还活着" | 对端迟迟发现不了你挂了 |

**History / Depth 不参与兼容性判断**，只影响本地缓存——这是最容易被误解的一点。

---

## 2. 兼容性规则：请求的 ≤ 提供的

一句话：**订阅端只能"要求得更弱"或"一样强"，不能要求更强。**

| 发布端 | 订阅端 | 能连上？ | 为什么 |
|---|---|---|---|
| `reliable` | `reliable` | ✅ | 一样 |
| `reliable` | `best_effort` | ✅ | 订阅端要求更弱，可以降级使用 |
| `best_effort` | `reliable` | ❌ | 订阅端要求重传，发布端给不了 |
| `volatile` | `volatile` | ✅ | 一样 |
| `volatile` | `transient_local` | ❌ | 订阅端要历史，发布端不存 |
| `transient_local` | `volatile` | ✅ | 订阅端不要历史 |
| deadline 100 ms | deadline 200 ms | ✅ | 发布端更严格 |
| deadline 200 ms | deadline 100 ms | ❌ | 发布端满足不了订阅端的承诺 |

连不上的时候**不会报错**，只是永远不 matched——这是最坑的地方：程序不崩，就是没数据。

---

## 3. 实验：把矩阵跑一遍

`qos_lab` 里 `qos_matrix` 一个进程同时起 pub + sub，两端 QoS 都由参数给。每秒打印一行：

```
[stats] matched pubs=1 subs=1 | sent=120 recv=120 lost=0 | max_gap=100.3 ms deadline_viol=0
```

跑法见本轮对话里的命令块。要观察的只有三列：`matched`、`lost`、`deadline_viol`。

| 组合 | 预期 matched | 说明 |
|---|---|---|
| 默认（都 reliable/volatile） | 1 / 1 | 基线 |
| pub best_effort + sub reliable | 0 / 0 | 不兼容，永远不匹配 |
| pub reliable + sub best_effort | 1 / 1 | 兼容，订阅端降级 |
| pub volatile + sub transient_local | 0 / 0 | 不兼容 |
| 100 Hz + depth 1 + `sub_busy_ms:=50` | 1 / 1，`lost` 快速增长 | 订阅端慢，队列只有 1 格，被覆盖 |
| rate 5 Hz + `sub_deadline_ms:=100` | 1 / 1，`deadline_viol` 每秒 +5 | 间隔 200 ms > 承诺 100 ms |

---

## 4. 三个容易误解的点

1. **`transient_local` 不是"所有历史都给你"**：只给 `depth` 条，而且过期（lifespan）的不给。`lifespan_lab` 就是用来打这个脸的。
2. **`depth` 是本地缓存，不是网络队列**：它决定"来不及消费时保留几条"，不决定"重传几次"。
3. **`reliable` 不等于不丢**：RELIABLE + depth 1 + 高频，照样丢。reliable 保证的是"**链路层重传**"，不是"**应用层不丢**"——队列满了照样覆盖。

---

## 5. 代码结构

- `qos_utils`：把 `<prefix>reliability/durability/history/depth` 翻译成 `rclcpp::QoS`，并打印。静态库，不导出——顺便复习 L0 里"包内复用 vs 跨包导出"的区别。
- `qos_matrix`：一个进程两端的实验台。
- `lifespan_lab`：晚加入 + 过期。

---

## 6. 自检 6 题

1. 订阅端 `reliable`、发布端 `best_effort`，为什么 `ros2 topic hz` 一直显示 no messages？
2. 地图话题用 `transient_local`，晚加入的节点为什么能立刻拿到一帧？需要满足哪两个条件？
3. `depth=1` 的 RELIABLE 发布端，在订阅端卡顿时会阻塞还是丢包？
4. `deadline` 是谁对谁的承诺？违约时谁会收到通知？
5. `lifespan` 和 `depth` 都能让数据"消失"，区别是什么？
6. 为什么 `matched` 为 0 时，节点既不报错也不退出？

（答案在文末）

---

## 7. 事件回调（先读，不写）

QoS 事件在 rclcpp 里是通过 **options 里的回调**注册的，不是通过派生类：

```cpp
// 伪代码，参数类型以你本机头文件为准
rclcpp::PublisherOptions opts;
opts.event_callbacks.incompatible_qos_callback = [](auto & info) { ... };
auto pub = create_publisher<T>(topic, qos, opts);
```

去 `/opt/ros/humble/include/rclcpp/rclcpp/publisher_options.hpp` 和 `subscription_options.hpp` 里找 `event_callbacks`，看它定义了哪几个回调、参数类型叫什么。**等你看清楚了再写**——这比自己猜 API 靠谱。

---

## 附：自检题答案

1. 兼容性不满足，DDS 根本不会建立 reader-writer 匹配，所以没有数据，也不报错。
2. 发布端 `transient_local` + `depth > 0`；且历史样本没有超过 `lifespan`。
3. 取决于实现与 QoS 组合：depth 满时 RELIABLE 可能阻塞发布端（背压）或按实现丢弃；BEST_EFFORT 一定丢。这正是 `bigdata_pool_benchmark_loss` 那类实验要测的东西。
4. 发布端对订阅端承诺"最长间隔"；违约时发布端收到 `offered_deadline_missed`，订阅端收到 `requested_deadline_missed`。
5. `depth` 限制**条数**，`lifespan` 限制**时长**；两者独立生效，任一满足就丢。
6. 因为兼容性是**静默失败**：DDS 只是在发现阶段判定不匹配，不产生错误码。
