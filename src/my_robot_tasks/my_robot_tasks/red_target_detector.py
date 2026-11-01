#!/usr/bin/env python3
"""检测画面中的红色目标，反投影到地图坐标并发布 /detected_target。

坐标链路：
    pixel  --K^-1-->  相机光学系方向
           --R_opt--> camera_link 方向
           --TF-----> 父坐标系(map/odom) 方向
           --与 z=0 求交-->  目标在地面上的位置

三个关键细节：
  1) 投影用【底部边缘】而不是整块重心。
     箱子重心常高于相机（我们相机只有 0.08 m），射线朝上就永远交不到地面。
  2) TF 查询用【图像自身的时间戳】而不是"最新"。
     车旋转时两者差一个帧间隔，会引入 d·ω·Δt 的横向误差。
  3) 用「整块像素」判面积、用「底部带状区域」判位置。
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformException
from rclpy.parameter import Parameter


# 相机光学坐标系（x右 / y下 / z前）-> 车体坐标系（x前 / y左 / z上）
R_OPTICAL_TO_LINK = np.array([[0.0, 0.0, 1.0],
                              [-1.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0]])


def quat_to_matrix(q):
    """四元数 -> 3x3 旋转矩阵"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


class RedTargetDetector(Node):
    def __init__(self):
        super().__init__('red_target_detector')

        # ---- 参数 ----
        self.min_area = int(self.declare_parameter('min_area', 300).value)
        self.r_min = int(self.declare_parameter('r_min', 60).value)
        self.color_diff = int(self.declare_parameter('color_diff', 40).value)
        self.bottom_band = int(self.declare_parameter('bottom_band', 5).value)
        self.parent_frame = str(self.declare_parameter('parent_frame', 'map').value)
        self.camera_frame = str(self.declare_parameter('camera_frame', 'camera_link').value)
        self.tf_backoff = float(self.declare_parameter('tf_time_backoff', 0.02).value)
        self.report_step = float(self.declare_parameter('report_step', 0.05).value)

        # ---- 状态 ----
        self.K = None
        self.last_map = None
        self.n_ok = 0
        self.n_tf_fail = 0
        self.n_proj_fail = 0

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseStamped, '/detected_target', 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)

        self.tf_mode = 'none'

        self.get_logger().info(
            'red_target_detector 启动  父坐标系=%s  相机=%s'
            % (self.parent_frame, self.camera_frame))

    # ------------------------------------------------------------------
    def on_info(self, msg):
        if self.K is not None:
            return
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.get_logger().info(
            '内参 fx=%.1f fy=%.1f cx=%.1f cy=%.1f'
            % (self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]))

    # ------------------------------------------------------------------
    def on_image(self, msg):
        if self.K is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        b = img[:, :, 0].astype(np.int16)
        g = img[:, :, 1].astype(np.int16)
        r = img[:, :, 2].astype(np.int16)

        # 红色判定：R 足够高，且明显高于 G、B
        mask = ((r > self.r_min)
                & (r - g > self.color_diff)
                & (r - b > self.color_diff))
        n_px = int(mask.sum())
        if n_px < self.min_area:
            return

        ys, xs = np.nonzero(mask)

        # ★ 底部带状区域的中点：真正贴地的那条边
        v_bottom = int(ys.max())
        band = ys >= v_bottom - self.bottom_band
        u = float(xs[band].mean())
        v = float(v_bottom)

        p = self.project_to_ground(u, v, msg.header.stamp)
        if p is None:
            return

        out = PoseStamped()
        out.header.frame_id = self.parent_frame
        out.header.stamp = msg.header.stamp
        out.pose.position.x = p[0]
        out.pose.position.y = p[1]
        out.pose.orientation.w = 1.0
        self.pub.publish(out)

        self.n_ok += 1
        moved = (self.last_map is None or
                 np.hypot(p[0] - self.last_map[0], p[1] - self.last_map[1]) > self.report_step)
        if moved:
            self.last_map = p
            self.get_logger().info(
                '红块 %5d px 底边@(%.0f,%.0f) -> %s (%.2f, %.2f)  '
                '[tf=%s ok=%d tf失败=%d 投影失败=%d]'
                % (n_px, u, v, self.parent_frame, p[0], p[1],
                   self.tf_mode, self.n_ok, self.n_tf_fail, self.n_proj_fail))


    def lookup_camera(self, stamp):
        """查 map->camera_link。

        先按【图像自己的时间戳】查（几何上正确）；
        失败则退回【最新 TF】（会引入 d·ω·Δt 的旋转滞后误差，但总能出结果）。
        相机时间戳可能比 TF 更新，所以必须有这个回退。
        """
        candidates = [
            ('stamped', Time.from_msg(stamp) - Duration(seconds=self.tf_backoff)),
            ('latest', Time()),
        ]
        last_err = None
        for tag, tq in candidates:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.parent_frame, self.camera_frame, tq,
                    timeout=Duration(seconds=0.15))
                self.tf_mode = tag
                return tf
            except TransformException as e:
                last_err = e

        self.n_tf_fail += 1
        msg = str(last_err)
        if 'does not exist' in msg:
            self.get_logger().warn(
                '坐标系 "%s" 不存在 —— 起来 Nav2/AMCL 了吗？(或 -p parent_frame:=odom)'
                % self.parent_frame, throttle_duration_sec=5.0)
        else:
            self.get_logger().warn('TF 查询失败: %s' % msg, throttle_duration_sec=5.0)
        return None

    # ------------------------------------------------------------------
    def project_to_ground(self, u, v, stamp):
        """像素 (u,v) -> 父坐标系下与地面 z=0 的交点"""
        # ① 像素 -> 相机光学系方向
        d_opt = np.linalg.solve(self.K, np.array([u, v, 1.0]))
        d_opt /= np.linalg.norm(d_opt)

        # ② 相机位姿：用【图像自己的时间戳】，稍往过去退一点避免外推
        tf = self.lookup_camera(stamp)
        if tf is None:
            return None

        tr = tf.transform.translation
        R = quat_to_matrix(tf.transform.rotation)

        # ③ 光学系 -> 父坐标系
        d = R @ (R_OPTICAL_TO_LINK @ d_opt)

        # ④ 与地面 z = 0 求交
        if d[2] >= -1e-6:
            self.n_proj_fail += 1
            self.get_logger().warn(
                '射线朝上/平行地面 (dz=%.3f)，无法与 z=0 相交 —— 检查是否投到了物体中上部'
                % d[2], throttle_duration_sec=5.0)
            return None
        s = -tr.z / d[2]
        if s <= 0.0:
            self.n_proj_fail += 1
            return None

        return (tr.x + s * d[0], tr.y + s * d[1])


def main(args=None):
    rclpy.init(args=args)
    node = RedTargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()