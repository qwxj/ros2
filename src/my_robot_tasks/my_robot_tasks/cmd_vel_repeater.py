#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRepeater(Node):
    """把「一次性」速度指令变成「持续」指令流。

    键盘遥控每按一次只发一条消息，而 cmd_vel_watchdog 要求 0.5 秒内
    必须收到新指令否则停车。本节点以固定频率重复最后一条指令。
    """

    def __init__(self):
        super().__init__('cmd_vel_repeater')

        # 参数
        self.rate_hz = float(self.declare_parameter('rate_hz', 10.0).value)
        self.source_timeout = float(
            self.declare_parameter('source_timeout', 5.0).value)

        # 状态
        self.cmd = Twist()
        self.last_rx = None
        self.has_cmd = False

        # ★ 第一次用 rclpy 发布
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(
            f'cmd_vel_repeater: /cmd_vel_teleop -> /cmd_vel @ '
            f'{self.rate_hz:.0f} Hz，源超时 {self.source_timeout:.1f} s')

    def on_cmd(self, msg):
        self.cmd = msg
        self.last_rx = self.get_clock().now()
        self.has_cmd = True

    def tick(self):
        if not self.has_cmd:
            return
        age = (self.get_clock().now() - self.last_rx).nanoseconds / 1e9
        if age > self.source_timeout:
            self.cmd = Twist()      # 源太久没更新 → 发零
        self.pub.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRepeater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()