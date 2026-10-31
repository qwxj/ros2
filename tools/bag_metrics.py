#!/usr/bin/env python3
"""从 rosbag2 读取一段导航过程，输出客观指标，用于参数对比。

用法：
    python3 tools/bag_metrics.py bags/ctrl_v015 bags/ctrl_v030

指标说明：
    时长         首末消息时间差
    |v| 平均/最大  线速度统计（最大 ≈ 参数 max_vel_x，除非被 velocity_smoother 限幅）
    真停占比       |v|=0 且 |w|=0       —— 真的停着
    原地转占比     |v|=0 且 |w|>0       —— 停在原地转向
    抖动/样本     相邻速度差之和 ÷ 采样数（已归一化，可跨 bag 比较）

注意（离线回放场景）：
    /plan 的长度与形状  ✅ 反映 planner 参数
    /cmd_vel 的曲线     ✅ 反映 controller 参数
    /amcl_pose 的轨迹   ⚠️ 离线时由 bag 决定；在线时才反映真实行驶距离
"""
import math
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


def read_bag(path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {'twist': [], 'pose': [], 'scan': [], 'plan': [], 't0': None, 't1': None}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in types:
            continue
        cls = get_message(types[topic])
        msg = deserialize_message(data, cls)
        # Humble 的 read_next 返回 int 纳秒；兼容 Time 对象
        t = stamp / 1e9 if isinstance(stamp, int) else stamp.nanoseconds / 1e9
        if out['t0'] is None:
            out['t0'] = t
        out['t1'] = t
        if topic == '/cmd_vel':
            out['twist'].append((t, msg.linear.x, msg.angular.z))
        elif topic == '/amcl_pose':
            p = msg.pose.pose.position
            out['pose'].append((t, p.x, p.y))
        elif topic == '/scan':
            rng = [x for x in msg.ranges if math.isfinite(x) and x > 0.01]
            out['scan'].append((t, min(rng) if rng else float('nan')))
        elif topic == '/plan':
            pts = [(q.pose.position.x, q.pose.position.y) for q in msg.poses]
            out['plan'].append((t, pts))
    return out


def poly_len(pts):
    return sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
               for i in range(1, len(pts)))


def report(path):
    d = read_bag(path)
    print("=" * 66)
    print("bag: %s" % path)

    if d['t0'] is not None and d['t1'] is not None:
        print("  ★ 时长           : %.1f s（首末消息）" % (d['t1'] - d['t0']))

    # ---- 全局路径 ----
    if d['plan']:
        lens = [poly_len(pts) for t, pts in d['plan'] if len(pts) > 1]
        if lens:
            print("  ★ 全局路径       : %d 条，首条 %.2f m / 最后 %.2f m / 平均 %.2f m"
                  % (len(lens), lens[0], lens[-1], sum(lens) / len(lens)))
            print("  ★ 路径点数       : 首条 %d / 最后 %d"
                  % (len(d['plan'][0][1]), len(d['plan'][-1][1])))
    else:
        print("  ★ 全局路径       : —（没有 /plan）")

    # ---- 速度 ----
    if d['twist']:
        vs = [abs(v) for t, v, w in d['twist']]
        ws = [abs(w) for t, v, w in d['twist']]
        n = float(len(vs))
        stop = sum(1 for v, w in zip(vs, ws) if v <= 1e-3 and w <= 0.05)
        spin = sum(1 for v, w in zip(vs, ws) if v <= 1e-3 and w > 0.05)
        jerk = sum(abs(vs[i] - vs[i - 1]) for i in range(1, len(vs)))
        print("  ★ 速度采样       : %d 条（约 %.1f s @20Hz）" % (len(vs), len(vs) / 20.0))
        print("  ★ |v| 平均/最大  : %.3f / %.3f m/s" % (sum(vs) / n, max(vs)))
        print("  ★ |w| 平均/最大  : %.3f / %.3f rad/s" % (sum(ws) / n, max(ws)))
        print("  ★ 真停占比       : %.1f%%（|v|=0 且 |w|=0）" % (100.0 * stop / n))
        print("  ★ 原地转占比     : %.1f%%（|v|=0 且 |w|>0）" % (100.0 * spin / n))
        print("  ★ 抖动/样本      : %.4f m/s（已归一化，可跨 bag 比较）" % (jerk / max(n - 1, 1)))

    # ---- 轨迹 ----
    if len(d['pose']) > 1:
        L = poly_len([(x, y) for t, x, y in d['pose']])
        print("    /amcl_pose 轨迹: %.2f m（离线时由 bag 决定；在线时是真实行驶距离）" % L)

    # ---- 激光 ----
    sc = [v for t, v in d['scan'] if not math.isnan(v)]
    if sc:
        print("    /scan 最近距离 : %.2f m（平均 %.2f）" % (min(sc), sum(sc) / len(sc)))
    print("")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for p in sys.argv[1:]:
        try:
            report(p)
        except Exception as e:
            print("  ❌ %s 读取失败: %s" % (p, e))