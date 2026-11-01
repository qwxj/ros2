#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# 巡逻航点（已验证都在空闲区，最近障碍 ≥ 0.86 m）
WAYPOINTS = [
    (1.5, -1.2),
    (1.5, 1.2),
    (-1.0, 1.2),
    (-1.0, -1.2),
]

LAPS = 2        # 巡逻几圈
DWELL = 3.0     # 每圈之间停留几秒


def make_pose(x, y):
    """构造一个 map 坐标系下的目标点。

    注意：stamp 故意留空（全 0）——
    Nav2 会把 0 时间戳当作"使用最新可用的 TF"，
    避免节点时钟与仿真时钟不一致导致的 extrapolation 错误。
    """
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0     # yaw = 0
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()
    log = nav.get_logger()

    log.info('等待 Nav2 就绪（bt_navigator + amcl）...')
    nav.waitUntilNav2Active()
    log.info('Nav2 就绪，开始巡逻')

    poses = [make_pose(x, y) for x, y in WAYPOINTS]

    for lap in range(1, LAPS + 1):
        log.info(f'===== 第 {lap}/{LAPS} 圈 =====')

        if not nav.followWaypoints(poses):
            log.error('航点序列被拒绝，退出')
            break

        last_wp = -1
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None:
                wp = getattr(fb, 'current_waypoint', -1)
                if wp != last_wp:
                    last_wp = wp
                    log.info(f'  → 前往第 {wp + 1}/{len(poses)} 个航点')
            time.sleep(0.1)

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            log.info(f'第 {lap} 圈完成')
        else:
            log.warn(f'第 {lap} 圈未成功: {result}')

        if lap < LAPS:
            log.info(f'停留 {DWELL:.0f} 秒...')
            time.sleep(DWELL)

    log.info('巡逻结束')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()