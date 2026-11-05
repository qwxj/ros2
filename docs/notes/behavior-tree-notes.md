# Nav2 行为树与恢复行为

> 对应配置：`config/nav2_params_standalone.yaml`（`bt_navigator` 段）、`launch/nav2_standalone.launch.py`

## 1. 为什么用行为树

| | 状态机 | 行为树 |
|---|---|---|
| 改流程 | 改代码、重新编译 | 改 XML、重启即可 |
| 可视化 | 图与代码易脱节 | Groot 可实时查看运行节点 |
| 失败处理 | 需要到处写分支 | `Fallback` / `RecoveryNode` 原生支持 |
| 打断 | 困难 | 每次 tick 重新评估 |

状态机回答“我现在在哪个状态”，行为树回答“我现在该尝试哪条分支”。

## 2. tick 与三种返回

执行器按 `bt_loop_duration`（默认 10 ms）从根节点向下 tick，每个节点返回三者之一：

- `SUCCESS`：完成；
- `FAILURE`：失败；
- `RUNNING`：尚未完成，下一 tick 继续。

`RUNNING` 是行为树的关键：长任务（如 `FollowPath`）持续返回 `RUNNING`，不会阻塞整棵树，其他分支仍可被 tick。

## 3. 节点类型

| 类型 | 语义 |
|---|---|
| `Sequence` | 依次执行，任一 `FAILURE` 则整条失败 |
| `Fallback` | 依次尝试，任一 `SUCCESS` 则整条成功 |
| `PipelineSequence` | 前序成功后仍持续 tick（支持边走边重规划） |
| `RecoveryNode` | 主分支失败 → 执行恢复 → 重试主分支，带次数上限 |
| `RateController` | 限频（默认树中为 1 Hz，控制重规划频率） |
| 动作节点 | `ComputePathToPose`、`FollowPath`、`Spin`、`Wait`、`ClearEntireCostmap` |

## 4. 默认树结构

```
RecoveryNode（最多重试 N 次）
├── 主分支：PipelineSequence
│   ├── RateController(1 Hz) ──▶ ComputePathToPose
│   └── FollowPath
└── 恢复分支（依次尝试）
    ├── ClearEntireCostmap
    ├── Spin
    └── Wait
```

恢复顺序按成本从低到高：清代价地图几乎不耗时，旋转需要动底盘，等待最耗时。

## 5. 黑板

节点之间通过黑板传数据，不直接传参：

```
ComputePathToPose 写 {path}  →  FollowPath 读 {path}
```

XML 只描述依赖关系，数据流由黑板承担。自定义节点读写同样的键。

## 6. 自定义流程

1. 复制默认 XML：`navigate_to_pose_w_replanning_and_recovery.xml`；
2. 修改结构（增加分支、调整顺序、修改等待时长）；
3. 设置参数 `default_nav_to_pose_bt_xml` 指向新文件。

若需要新的节点类型：继承 `BtActionNode` → pluginlib 导出 → 加入 `plugin_lib_names`。

## 7. 调试手段

| 手段 | 用途 |
|---|---|
| `ros2 action send_goal ... --feedback` | 查看 `number_of_recoveries`、剩余距离 |
| Groot | 可视化当前处于 `RUNNING` 的节点 |
| `/behavior_tree_log` | 每次 tick 的节点状态 |
| `bt_navigator` 日志 | 判断卡在哪一层 |

判据：导航“成功但恢复多次”说明参数或代价地图有问题；“失败且恢复次数为 0”说明未进入恢复分支（多半是 BT XML 或插件清单配置问题）。
