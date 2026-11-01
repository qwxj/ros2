#!/usr/bin/env python3
"""订阅 /detected_target，收敛后导航过去。

四个关键点：
  1) 继承 BasicNavigator —— 它内部会自己 spin_until_future_complete，
     两个 Node 同时 spin 容易死锁。
  2) 开 use_sim_time —— BasicNavigator 默认 false，会让目标时间戳变成墙钟
     （墙钟 ~1.8e9 秒 vs 仿真钟 ~260 秒，Nav2 收到的是"56 年后的目标"）。
  3) 目标点回退 standoff —— 视觉目标同时是物理障碍，它的表面落在
     代价地图的致命膨胀区里，直接导航过去会被拒绝。
  4) 不调 waitUntilNav2Active() —— 它内部阻塞等 /amcl_pose，
     AMCL 粒子滤波没更新时会永久卡在"等 Nav2 就绪"。
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf2_ros


class GoToTarget(BasicNavigator):
    def __init__(self):
        super().__init__()
        # ★ 改动 1：开仿真时间
        self.set_parameters([Parameter('use_sim_time', value=True)])

        self.min_hits = int(self.declare_parameter('min_hits', 5).value)
        self.spread_tol = float(self.declare_parameter('spread_tol', 0.20).value)
        self.standoff = float(self.declare_parameter('standoff', 0.45).value)

        self.hits = []
        self.started = False
        self.finished = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.log = self.get_logger()
        self.create_subscription(PoseStamped, '/detected_target', self.on_target, 10)
        self.log.info('等待 /detected_target ... （工作距离 %.2f m）' % self.standoff)

    # ------------------------------------------------------------------
    def on_target(self, msg):
        if self.started:
            return

        self.hits.append((msg.pose.position.x, msg.pose.position.y))
        n = len(self.hits)
        gx = sum(h[0] for h in self.hits) / n
        gy = sum(h[1] for h in self.hits) / n
        spread = max(math.hypot(h[0] - gx, h[1] - gy) for h in self.hits)

        self.log.info('第 %d/%d 帧  (%.2f, %.2f)  抖动 %.3f m'
                      % (n, self.min_hits, gx, gy, spread))

        if n < self.min_hits:
            return
        if spread > self.spread_tol:
            self.log.warn('抖动太大 %.2f m，丢弃重来' % spread)
            self.hits = []
            return

        rxry = self.robot_xy()
        if rxry is None:
            # 不在回调里 spin，直接等下一帧重试
            self.log.warn('还拿不到 map->base_footprint，下一帧再试')
            self.hits = []
            return

        rx, ry = rxry
        dist = math.hypot(gx - rx, gy - ry)

        # 自适应工作距离：不能退到车屁股后面去
        eff = min(self.standoff, max(0.30, dist - 0.50))
        self.log.info('车到目标 %.2f m，实际回退 %.2f m（上限 %.2f）'
                      % (dist, eff, self.standoff))

        dx, dy = gx - rx, gy - ry
        if dist > 1e-3:
            gx -= eff * dx / dist
            gy -= eff * dy / dist

        self.start(gx, gy, rxry)

    # ------------------------------------------------------------------
    def robot_xy(self):
        for tq, tag in ((rclpy.time.Time(), 'latest'),):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', tq,
                    timeout=Duration(seconds=0.5))
                return tf.transform.translation.x, tf.transform.translation.y
            except Exception as e:
                self.log.warn('取车位置失败(%s): %s' % (tag, str(e)[:70]))
        return None

    # ------------------------------------------------------------------
    def start(self, tx, ty, rxry):
        self.started = True

        rx, ry = rxry
        dx, dy = tx - rx, ty - ry
        L = math.hypot(dx, dy)
        gx, gy = tx, ty
        if L > self.standoff:
            gx = tx - self.standoff * dx / L
            gy = ty - self.standoff * dy / L
        self.log.info('目标 (%.2f, %.2f) 从车 (%.2f, %.2f) 方向回退 %.2f m'
                      % (tx, ty, rx, ry, self.standoff))

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        # ★ 改动 3：stamp 留 0，Nav2 当作"用最新 TF"
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation.w = 1.0

        self.log.info('导航目标 (%.2f, %.2f)，等 bt_navigator 上线...' % (gx, gy))
        # ★ 改动 3：不调 waitUntilNav2Active()，goToPose 内部会等 action server
        self.goToPose(goal)
        self.log.info('已下发目标，开始导航')

        self.create_timer(0.5, self.check)

    # ------------------------------------------------------------------
    def check(self):
        if self.finished:
            return
        if self.result_future is None or not self.result_future.done():
            return
        self.finished = True
        st = self.result_future.result().status
        if st == 4:                       # STATUS_SUCCEEDED
            self.log.info('✅ 到达目标（status=%d 成功）' % st)
        elif st == 5:
            self.log.warn('❌ 被取消 status=5')
        elif st == 6:
            self.log.warn('❌ 放弃 status=6')
        else:
            self.log.warn('❌ status=%d' % st)


def main(args=None):
    rclpy.init(args=args)
    node = GoToTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()