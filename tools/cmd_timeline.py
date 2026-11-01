#!/usr/bin/env python3
"""把 /cmd_vel 画成 0.5 秒一格的时间线，定位「真停 / 原地转 / 前进」发生在哪个阶段。

用法：
    python3 tools/cmd_timeline.py bags/ctrl_v030_fix bags/ctrl_norotate

图例：
    .   真停    |v|≈0 且 |w|≈0
    R   原地转  |v|≈0 且 |w|>0
    =   前进    或边走边转

为什么需要它：
    只看「原地转占比 21%」这个数字，无法判断它发生在起步、中途还是到点后 ——
    连续改错两轮 critic 权重之后才明白，必须先定位「哪一段」。
"""
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

BUCKET = 0.5  # 秒/格


def read_twist(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='sqlite3'),
                ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic != '/cmd_vel':
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        t = stamp / 1e9 if isinstance(stamp, int) else stamp.nanoseconds / 1e9
        out.append((t, msg.linear.x, msg.angular.z))
    return out


def main(paths):
    for path in paths:
        tw = read_twist(path)
        print("=" * 66)
        print("bag: %s   （%.1f s/格）" % (path, BUCKET))
        if not tw:
            print("  没有 /cmd_vel")
            print("")
            continue
        t0 = tw[0][0]
        buckets = {}
        for t, v, w in tw:
            k = int((t - t0) / BUCKET)
            buckets.setdefault(k, []).append((abs(v), abs(w)))
        line = []
        for k in sorted(buckets):
            vs = [v for v, _ in buckets[k]]
            ws = [w for _, w in buckets[k]]
            av, aw = sum(vs) / len(vs), sum(ws) / len(ws)
            if av < 0.02 and aw < 0.05:
                line.append('.')
            elif av < 0.02:
                line.append('R')
            else:
                line.append('=')
        s = "".join(line)
        n = max(len(s), 1)
        print("  " + s)
        print("  真停   %2d 格 = %4.1f s（%3.0f%%）" % (s.count('.'), s.count('.') * BUCKET, 100.0 * s.count('.') / n))
        print("  原地转 %2d 格 = %4.1f s（%3.0f%%）" % (s.count('R'), s.count('R') * BUCKET, 100.0 * s.count('R') / n))
        print("  前进   %2d 格 = %4.1f s（%3.0f%%）" % (s.count('='), s.count('=') * BUCKET, 100.0 * s.count('=') / n))
        print("")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1:])