#!/usr/bin/env python3
"""ros2_control 标准话题适配器。

背景
----
ros2_control 的每个控制器都是独立节点，话题用「私有命名」：

    diff_drive_controller 订阅  ~/cmd_vel_unstamped
                          发布  ~/odom

节点名是 diff_drive_controller、命名空间是 /，所以实际话题是：

    /diff_drive_controller/cmd_vel_unstamped
    /diff_drive_controller/odom

而 Nav2 / teleop / 录包工具用的都是标准的 /cmd_vel 和 /odom。
本节点做双向转发，让上层只用标准话题，不关心底层是 ros2_control 还是 Gazebo。

参数
----
input_topic   下行入口话题，默认 /cmd_vel_final
              （接了安全链之后，真正的「最后一道」是 collision_monitor 的输出；
               如果是裸跑没有安全链，把它设成 /cmd_vel 即可）

注意
----
- use_stamped_vel: false 时订阅名是 cmd_vel_unstamped；
  改成 true  则是 cmd_vel，类型变成 TwistStamped，需要改本节点。
- 如果同时跑 Gazebo 版底盘（也会发 /odom），两者会抢同一个话题，二选一。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class CmdVelAdapter(Node):
    def __init__(self):
        super().__init__('cmd_vel_adapter')

        self.declare_parameter('input_topic', '/cmd_vel_final')
        in_topic = self.get_parameter('input_topic').value

        # ── 下行：(可配置话题)  ->  /diff_drive_controller/cmd_vel_unstamped ──
        self.pub_cmd = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.create_subscription(Twist, in_topic, self.on_cmd, 10)

        # ── 上行：/diff_drive_controller/odom  ->  /odom ──
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.on_odom, 10)

        self.n_cmd = 0
        self.n_odom = 0
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            'cmd_vel_adapter 已启动：\n'
            '  下行 %s -> /diff_drive_controller/cmd_vel_unstamped\n'
            '  上行 /diff_drive_controller/odom -> /odom' % in_topic)

    def on_cmd(self, msg: Twist):
        self.pub_cmd.publish(msg)
        self.n_cmd += 1

    def on_odom(self, msg: Odometry):
        self.pub_odom.publish(msg)
        self.n_odom += 1

    def report(self):
        if self.n_cmd or self.n_odom:
            self.get_logger().info(
                '转发统计：下行 %d 条 上行 %d 条（最近 5 秒）' % (self.n_cmd, self.n_odom))
        self.n_cmd = 0
        self.n_odom = 0


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
