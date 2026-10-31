#!/usr/bin/env python3
"""从 rosbag2 读取一段导航过程，输出客观指标，用于参数对比。

用法：
    python3 tools/bag_metrics.py bags/tune_out_base bags/tune_out_inf035

注意：在离线回放场景下，
  - /plan 的长度与形状  ✅ 反映 planner 参数的效果
  - /cmd_vel 的曲线     ✅ 反映 controller 参数的效果
  - /amcl_pose 的轨迹   ❌ 由 bag 决定，改参数不会变（仅供参考）
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
    out = {'twist': [], 'pose': [], 'scan': [], 'plan': []}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in types:
            continue
        cls = get_message(types[topic])
        msg = deserialize_message(data, cls)
        t = stamp / 1e9 if isinstance(stamp, int) else stamp.nanoseconds / 1e9
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
    print("=" * 64)
    print("bag: %s" % path)

    # ---- 全局路径（最有意义）----
    if d['plan']:
        lens = [poly_len(pts) for t, pts in d['plan'] if len(pts) > 1]
        if lens:
            print("  ★ 全局路径条数   : %d" % len(lens))
            print("  ★ 路径长度       : 首条 %.2f m / 最后 %.2f m / 平均 %.2f m"
                  % (lens[0], lens[-1], sum(lens) / len(lens)))
            print("  ★ 路径点数       : 首条 %d / 最后 %d"
                  % (len(d['plan'][0][1]), len(d['plan'][-1][1])))
    else:
        print("  ★ 全局路径       : —（没有 /plan）")

    # ---- 速度指令 ----
    tw = [(t, v, w) for t, v, w in d['twist']]
    if tw:
        vs = [abs(v) for t, v, w in tw]
        ws = [abs(w) for t, v, w in tw]
        zero_ratio = 100.0 * sum(1 for v in vs if v <= 1e-3) / len(vs)
        # 平滑度：相邻样本的速度变化量之和（越小越平滑）
        jerk = sum(abs(vs[i] - vs[i - 1]) for i in range(1, len(vs)))
        print("  ★ 速度采样       : %d 条" % len(tw))
        print("  ★ |v| 平均/最大  : %.3f / %.3f m/s" % (sum(vs) / len(vs), max(vs)))
        print("  ★ |w| 平均/最大  : %.3f / %.3f rad/s" % (sum(ws) / len(ws), max(ws)))
        print("  ★ 零速占比       : %.1f%%" % zero_ratio)
        print("  ★ 速度抖动       : %.2f（相邻速度差之和，越小越平滑）" % jerk)

    # ---- 回放轨迹（仅供参考）----
    if len(d['pose']) > 1:
        L = poly_len([(x, y) for t, x, y in d['pose']])
        print("    /amcl_pose 轨迹: %.2f m（⚠️ 由 bag 决定，不反映参数差异）" % L)

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