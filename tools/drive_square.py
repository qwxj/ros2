#!/usr/bin/env python3
"""走一个闭合方形，用于触发回环（loop closure）。

为什么用 Python 而不是 `ros2 topic pub`：
    `ros2 topic pub` 是 CLI 程序，每次启动约 1~2 秒、退出也有开销。
    分段发指令时，两次调用之间的空档会超过 watchdog 的 0.5 s 超时，
    车会被反复急停，走不出完整轨迹。
    单进程连续 publish 就没有这个空档。

用法：
    python3 tools/drive_square.py                       # 默认：1.5 m 方形（左转）
    python3 tools/drive_square.py --side 1.0            # 走小一点
    python3 tools/drive_square.py --speed 0.1           # 走慢一点
    python3 tools/drive_square.py --turn-dir -1         # 顺时针
    python3 tools/drive_square.py --laps 2              # 走两圈

    # 推荐：走仿真场景里的无障碍矩形（避开中间那四块障碍物）
    python3 tools/drive_square.py --legs 2.5,2.0,1.45,2.0 --turn-deg -90

注意：默认的 1.5 m 方形会撞上场景里的障碍物（C 段 x 1.2~2.85 / y 0.5~1.2），
      在 my_robot_description 的默认世界里有风险，请用上面的 --legs 版本。

前置条件：
    1. 底盘已启动：  ros2 launch my_robot_control robot.launch.py
    2. SLAM 已启动：  ros2 run mini_slam slam_mini_node --ros-args ...
"""
import argparse
import math
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--side', type=float, default=1.5,
                    help='方形边长（米），默认 1.5；指定 --legs 时忽略')
    ap.add_argument('--legs', default=None,
                    help='任意段长，逗号分隔，例如 2.5,2.0,1.45,2.0。'
                         '每段结束后按 --turn-deg 转向。指定后忽略 --side')
    ap.add_argument('--turn-deg', type=float, default=90.0,
                    help='每段后转的角度，负数=右转（顺时针），默认 90（左转）')
    ap.add_argument('--speed', type=float, default=0.15,
                    help='直行速度（m/s），默认 0.15')
    ap.add_argument('--yaw-rate', type=float, default=0.40,
                    help='转向角速度（rad/s），默认 0.40')
    ap.add_argument('--turn-dir', type=int, default=1, choices=[1, -1],
                    help='1=左转（逆时针），-1=右转（顺时针）')
    ap.add_argument('--laps', type=int, default=1, help='走几圈')
    ap.add_argument('--topic', default='/cmd_vel', help='速度话题')
    ap.add_argument('--settle', type=float, default=1.5,
                    help='开始前等待发现的时间（秒）')
    args = ap.parse_args()

    # 参数解析放在导入 rclpy 之前 —— 这样没 source ROS 时
    # `--help` 仍然能用，只在实际运行时才要求 ROS 环境。
    try:
        import rclpy
        from geometry_msgs.msg import Twist
    except ModuleNotFoundError:
        sys.exit(
            '找不到 rclpy。请先 source ROS 2 环境：\n'
            '    source /opt/ros/humble/setup.bash\n'
            '    source ~/ros2/install/setup.bash')

    rclpy.init()
    node = rclpy.create_node('square_driver')
    pub = node.create_publisher(Twist, args.topic, 10)
    time.sleep(args.settle)          # 等 DDS 发现

    if args.legs:
        legs = [float(v) for v in args.legs.split(',') if v.strip()]
    else:
        legs = [args.side] * 4
    turn_rad = math.radians(args.turn_deg)
    t_turn = abs(turn_rad) / args.yaw_rate

    def hold(vx, wz, dur):
        """持续发布同一速度指令 dur 秒，然后停稳。"""
        t0 = time.time()
        m = Twist()
        m.linear.x = vx
        m.angular.z = wz
        while time.time() - t0 < dur:
            pub.publish(m)
            time.sleep(0.05)
        stop = Twist()
        for _ in range(6):
            pub.publish(stop)
            time.sleep(0.05)

    total = args.laps * (sum(legs) / args.speed + len(legs) * t_turn)
    node.get_logger().info(
        '路径：%s 段 %s，每段后转 %.0f°，速度 %.2f m/s，%d 圈，预计 %.1f 秒'
        % (len(legs), legs, args.turn_deg, args.speed, args.laps, total))

    for lap in range(args.laps):
        for i, L in enumerate(legs):
            node.get_logger().info(
                '%d-%d 前进 %.2f m' % (lap + 1, i + 1, L))
            hold(args.speed, 0.0, L / args.speed)
            node.get_logger().info(
                '%d-%d 转 %.0f°' % (lap + 1, i + 1, args.turn_deg))
            hold(0.0, args.turn_dir * args.yaw_rate * (1 if args.turn_deg >= 0 else -1),
                 t_turn)

    node.get_logger().info('回到起点，结束')
    hold(0.0, 0.0, 1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
