# L1 通信层（二）：DDS、rmw 与发现机制

> **目标**：能解释"为什么跨机器收不到"、"为什么同机还要拷贝"、"为什么节点看得见但没数据"。
> **前置**：L1-QoS（知道兼容性规则）。

---

## 1. 分层：哪一层可以被替换

```
你的代码
  rclcpp        C++ API（Node / Publisher / Executor）
    rcl           C API，语言无关，行为由 DDS 决定
      rmw         中间件抽象层  ← 唯一可替换的地方
        rmw_fastrtps_cpp  /  rmw_cyclonedds_cpp
          DDS/RTPS 实现（Fast DDS、Cyclone DDS）
            UDP / 共享内存 / 组播
```

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 换的是 **rmw 之下**，上层代码一行不改。
- QoS 语义由 **DDS 规范**定义，所以换 RMW 不该改变语义，但**默认值、性能、发现行为**会变。

---

## 2. 发现：三阶段，顺序不能反

| 阶段 | 协议 | 干什么 | 结果 |
|---|---|---|---|
| 1 | **SPDP**（Participant Discovery） | 组播（默认 239.255.0.1:7400+250×domain）宣告"我存在" | `ros2 node list` 能看到对方 |
| 2 | **SEDP**（Endpoint Discovery） | 通过内置通道交换 writer/reader 的 topic、类型、**QoS** | 知道对端有哪些端点 |
| 3 | **匹配**（Matching） | 逐条比对 QoS 兼容性 | 匹配成功才建立数据流 |

**这就是"看得见节点、看不见数据"的根因**：SPDP/SEDP 都成功了，卡在第 3 步——QoS 不兼容，静默不匹配。

---

## 3. `ROS_DOMAIN_ID`：一个数字隔离一整个世界

- 端口算法：`7400 + 250 × domain_id`，所以**不同 domain 连组播端口都不同**，物理上互相收不到。
- 官方建议 `0–101`：再大就开始撞系统临时端口段（`7400+250×101=32650` 已接近）。
- 多机器人/多项目同网段共存，第一件事就是分 domain。
- 同一个 domain 内，**所有节点共享一个发现域**——节点多了，发现流量会成为负担（见 §5）。

```bash
echo $ROS_DOMAIN_ID                    # 空 = 0
ROS_DOMAIN_ID=7 ros2 topic list
```

---

## 4. 四条传输通路：拷贝账

同样是"发一条消息"，走的路径可以差出 2~3 次拷贝：

| 通路 | 序列化 | 跨进程 | 大致拷贝次数 | 什么时候用 |
|---|---|---|---|---|
| **intra-process**（composition 同容器） | 否 | 否 | 0 | 同进程节点之间 |
| **data_sharing**（loaned message） | 否 | 是（共享内存） | 0 | 同机、定长、大消息 |
| **SHM transport**（Fast DDS 默认共享内存） | 是 | 是 | 序列化 1 次 + 共享内存拷贝 | 同机普通消息 |
| **UDP** | 是 | 是 | 序列化 + 内核收发拷贝 | 跨机 |

- 你之前测的"1 MB @ 10 Hz，loaned 比普通低约 30%"，就是走了第 2 条而不是第 3 条。
- `data_sharing` 的前提：同 host、同 user、消息定长且 typesupport 支持 loan。
- 容器里注意 `/dev/shm` 大小（`--shm-size`），太小会让 SHM 走不通、静默退回 UDP。

---

## 5. 组播、单播与 Discovery Server

- 默认靠**组播**发现。交换机/防火墙/VPN/云主机常常禁组播 → 跨机直接失联。
- 没有组播时的两条路：
  1. **静态 peers**：把对端 IP 写进 DDS profile（Fast DDS 的 `initialPeersList`）。
  2. **Discovery Server**：一个中心节点做撮合，端点只连它（Fast DDS 特性，大规模系统常用）。
- `ROS_LOCALHOST_ONLY=1`：只在本机发现，适合"我不想被同网段别人看见"的调试。
- 注意：**发现走组播，数据走单播**——所以"能发现"不代表"数据能通"，防火墙要放行两端的高位 UDP 端口。

---

## 6. 故障树

| 现象 | 最可能原因 | 先查什么 |
|---|---|---|
| 跨机完全看不到对方 | 组播被拦 / 不同网段 / domain 不同 | 同 domain + 同网段直连对比；`ss -uapn \| grep 7400` |
| 本机、容器之间看不到 | 网络命名空间 / domain / localhost_only | 宿主机直接跑一遍对比 |
| 看得见节点，看不见话题数据 | QoS 不兼容 / 端点未创建 | `ros2 topic info -v` 看两端 QoS |
| 时通时断 | WiFi 组播不稳定 / MTU 分片 / SHM 权限 | 换有线；调 MTU；看 `/dev/shm` |
| 只有单向通 | QoS 方向性（一侧强一侧弱） | 两端 QoS 并排比对 |
| `ros2 node list` 卡住 | daemon 缓存 / 网络半死 | `ros2 daemon stop && ros2 daemon start` |

---

## 7. 命令清单（现场必备）

```bash
echo $ROS_DOMAIN_ID $RMW_IMPLEMENTATION $ROS_LOCALHOST_ONLY
ros2 daemon stop && ros2 daemon start
ros2 node list
ros2 topic info -v /your_topic
ss -uapn | grep 7400
ros2 doctor --report
ls -l /dev/shm | head
```

---

## 8. 实验（不需要写代码）

**实验 A · domain 隔离**

```bash
# 终端 1
ROS_DOMAIN_ID=7 ros2 topic pub -r 1 /demo std_msgs/msg/String "{data: hi}"

# 终端 2：能收到
ROS_DOMAIN_ID=7 ros2 topic echo /demo

# 终端 3：收不到（不同 domain = 不同组播端口）
ROS_DOMAIN_ID=8 ros2 topic echo /demo
```

**实验 B · 端口验证**

```bash
ss -uapn | grep -E '74[0-9][0-9]'
# 再起一个 domain=8 的节点，观察出现 7400+250*8=9400 附近的端口
```

**实验 C · 换 RMW**

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run qos_lab qos_matrix
```
对比启动耗时、`/dev/shm` 占用、大消息延迟。

**实验 D · 进程内 vs DDS**

```bash
ros2 run my_first_pkg bigdata_benchmark intra 1 10
ros2 run my_first_pkg bigdata_benchmark dds   1 10
```

---

## 9. 自检

1. 为什么"节点看得见、话题没数据"时，先看 QoS 而不是网络？
2. `ROS_DOMAIN_ID` 不同，为什么连发现都做不到（而不是发现了但连不上）？
3. 同机两条节点，什么时候可以做到零拷贝？需要哪三个条件？
4. 容器里 SHM 失效时，表现是什么？怎么确认走的是 UDP 还是 SHM？
5. 发现流量大时，除了分 domain 还有什么办法？

---

## 附：答案

1. 因为 SPDP/SEDP 成功只证明"存在于同一发现域"，第 3 步匹配失败是 QoS 语义问题，与网络无关。
2. domain 直接决定组播端口，端口不同 → 收不到 announce，连 participant 都发现不了。
3. 同进程（composition 或同节点）+ 开启 intra-process comms + 类型支持零拷贝。
4. 表现是延迟/CPU 升高但不报错；用 `ss`、`/dev/shm` 占用、`data_sharing` 日志确认，或直接对比吞吐。
5. Discovery Server（中心撮合）、静态 peers、减少端点数量、按域拆分。
