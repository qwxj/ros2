#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')

        # ① 参数（对照 C++ 的 declare_parameter）
        self.period = self.declare_parameter('print_period', 1.0).value

        # ② 成员状态
        self.latest = None
        self.last_xy = None
        self.total_dist = 0.0
        self.count = 0

        # ③ 订阅（对照 create_subscription<Odometry>）
        self.sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)

        # ④ 定时器（对照 create_wall_timer，单位是"秒"不是毫秒！）
        self.timer = self.create_timer(self.period, self.report)

        self.get_logger().info(
            f'pose_monitor 启动，每 {self.period:.1f} 秒汇报一次')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.latest = (p.x, p.y)
        self.count += 1
        if self.last_xy is not None:
            self.total_dist += math.hypot(p.x - self.last_xy[0],
                                          p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def report(self):
        if self.latest is None:
            self.get_logger().warning('还没有收到 /odom')
            return
        x, y = self.latest
        self.get_logger().info(
            f'位置 ({x:+.3f}, {y:+.3f})  累计里程 {self.total_dist:.3f} m  '
            f'{self.count} 帧')


def main(args=None):
    rclpy.init(args=args)
    node = PoseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()