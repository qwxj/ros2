#!/usr/bin/env python3
"""给 /odom 注入角速度偏差，模拟"打滑/轮径偏大"的坏里程计。

输出 /odom_noisy：
  - twist.angular.z 乘以 yaw_rate_scale（默认 1.10，即多报 10%）
  - pose.yaw 用【受污染后的角速度】重新积分，保证 twist 与 pose 自洽
  - 协方差原样保留（这样 EKF 仍更信 IMU）
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomNoise(Node):
    def __init__(self):
        super().__init__('odom_noise')

        self.scale = float(self.declare_parameter('yaw_rate_scale', 1.10).value)

        self.pub = self.create_publisher(Odometry, '/odom_noisy', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self.yaw = 0.0
        self.last_t = None

        self.get_logger().info(
            f'odom_noise: /odom -> /odom_noisy，角速度 ×{self.scale:.2f}')

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 用污染后的角速度积分 yaw（保证 twist 与 pose 自洽）
        if self.last_t is not None:
            dt = t - self.last_t
            if 0 < dt < 0.5:
                self.yaw += msg.twist.twist.angular.z * self.scale * dt
        self.last_t = t

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = msg.child_frame_id

        # 位置保持原样
        out.pose.pose.position.x = msg.pose.pose.position.x
        out.pose.pose.position.y = msg.pose.pose.position.y
        out.pose.pose.position.z = msg.pose.pose.position.z

        # 朝向用"污染后的积分值"
        out.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        out.pose.pose.orientation.w = math.cos(self.yaw * 0.5)

        # 速度：角速度乘系数
        out.twist.twist.linear.x = msg.twist.twist.linear.x
        out.twist.twist.linear.y = msg.twist.twist.linear.y
        out.twist.twist.angular.z = msg.twist.twist.angular.z * self.scale

        out.pose.covariance = list(msg.pose.covariance)
        out.twist.covariance = list(msg.twist.covariance)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()