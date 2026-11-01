# -*- coding: utf-8 -*-
"""视觉投影链路的单元测试。

跑法（不需要 Gazebo / Nav2）：
    cd ~/ros2/src/my_robot_tasks
    python3 -m pytest test/test_projection.py -v
"""
import math

import numpy as np
import pytest

from my_robot_tasks.geometry import (
    R_OPTICAL_TO_LINK,
    quat_xyzw_to_matrix,
    pixel_to_ray,
    optical_to_world,
    intersect_ground,
)

# 我们实际用的内参（从 /camera/camera_info 读到的）
K_REAL = np.array([[554.4, 0.0, 320.5],
                   [0.0, 554.4, 240.5],
                   [0.0, 0.0, 1.0]])


# ---------------------------------------------------------------- 旋转矩阵
def test_optical_to_link_axes():
    """光学系的三个轴必须映射到车体坐标系上正确的方向。"""
    optical_x = np.array([1.0, 0.0, 0.0])   # 图像向右
    optical_y = np.array([0.0, 1.0, 0.0])   # 图像向下
    optical_z = np.array([0.0, 0.0, 1.0])   # 镜头朝向

    # 图像向右 -> 车体 -Y（车体 y 轴朝左）
    assert np.allclose(R_OPTICAL_TO_LINK @ optical_x, [0, -1, 0])
    # 图像向下 -> 车体 -Z（车体 z 轴朝上）
    assert np.allclose(R_OPTICAL_TO_LINK @ optical_y, [0, 0, -1])
    # 镜头朝向 -> 车体 +X（车体 x 轴朝前）
    assert np.allclose(R_OPTICAL_TO_LINK @ optical_z, [1, 0, 0])


def test_optical_to_link_is_proper_rotation():
    """必须是真正的旋转（正交、行列式 +1），不能是镜像。"""
    R = R_OPTICAL_TO_LINK
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-12)


# ------------------------------------------------------------------ 四元数
def test_quat_identity():
    assert np.allclose(quat_xyzw_to_matrix(0, 0, 0, 1), np.eye(3))


def test_quat_yaw_90():
    """绕 Z 轴转 90°：车体 +X 应该转到 +Y。"""
    h = math.pi / 4      # 半角
    R = quat_xyzw_to_matrix(0, 0, math.sin(h), math.cos(h))
    assert np.allclose(R @ np.array([1.0, 0, 0]), [0, 1, 0], atol=1e-9)


# ------------------------------------------------------------------ 反投影
def test_pixel_center_gives_forward_ray():
    """主点（cx,cy）必须反投影成 (0,0,1) —— 正前方。"""
    d = pixel_to_ray(K_REAL[0, 2], K_REAL[1, 2], K_REAL)
    assert np.allclose(d, [0, 0, 1], atol=1e-12)


def test_pixel_ray_is_unit_length():
    for u, v in ((0, 0), (639, 479), (320, 240), (100, 400)):
        d = pixel_to_ray(u, v, K_REAL)
        assert math.isclose(np.linalg.norm(d), 1.0, abs_tol=1e-12)


def test_pixel_right_is_positive_x_in_optical():
    """图像右边的像素，光学系 x 分量应该为正。"""
    d = pixel_to_ray(K_REAL[0, 2] + 100, K_REAL[1, 2], K_REAL)
    assert d[0] > 0
    assert math.isclose(d[1], 0.0, abs_tol=1e-9)


# -------------------------------------------------------------- 与地面求交
def test_intersect_straight_down():
    """从 (0,0,0.08) 正下方射，应落在原点。"""
    p = intersect_ground([0, 0, 0.08], [0, 0, -1])
    assert p == pytest.approx((0.0, 0.0))


def test_intersect_45_degree():
    """45° 斜向下：水平位移 = 相机高度。"""
    d = np.array([1.0, 0.0, -1.0]) / math.sqrt(2)
    p = intersect_ground([0, 0, 0.08], d)
    assert p == pytest.approx((0.08, 0.0), abs=1e-9)


def test_intersect_upward_returns_none():
    """★ 射线朝上必须返回 None —— 这个断言能直接挡住今天的那个 bug
    （投箱子重心时射线朝上，与地面永不相交）"""
    assert intersect_ground([0, 0, 0.08], [0, 0, 1]) is None
    assert intersect_ground([0, 0, 0.08], [0.5, 0.0, 0.5]) is None


def test_intersect_parallel_returns_none():
    assert intersect_ground([0, 0, 0.08], [1.0, 0.0, 0.0]) is None


# ------------------------------------------------------------ 完整往返验证
def test_full_roundtrip():
    """已知一个地面点 -> 反算像素 -> 正投影回来 -> 必须回到原处。

    这是整条链路的双向自洽性检验。
    相机在 (0, 0, 0.08)，朝 +X，无旋转。
    """
    cam_origin = np.array([0.0, 0.0, 0.08])
    R_world_cam = np.eye(3)          # 无旋转
    target = np.array([1.0, -0.5, 0.0])

    # ① 世界系方向（从相机指向目标）
    dir_world = target - cam_origin
    dir_world /= np.linalg.norm(dir_world)

    # ② 世界系 -> 光学系：先去掉固定旋转，再去掉相机旋转
    d_opt = R_OPTICAL_TO_LINK.T @ (R_world_cam.T @ dir_world)

    # ③ 光学系方向 -> 像素
    uv = K_REAL @ d_opt
    u, v = uv[0] / uv[2], uv[1] / uv[2]

    # ④ 正投影回来
    d2 = pixel_to_ray(u, v, K_REAL)
    d2_world = optical_to_world(d2, R_world_cam)
    p = intersect_ground(cam_origin, d2_world)

    assert p is not None
    assert p == pytest.approx((1.0, -0.5), abs=1e-9)


def test_roundtrip_with_yaw():
    """相机转了 30° 也要往返一致。"""
    yaw = math.radians(30)
    R_world_cam = quat_xyzw_to_matrix(0, 0, math.sin(yaw/2), math.cos(yaw/2))
    cam_origin = np.array([0.5, 0.2, 0.08])
    target = np.array([2.0, 1.5, 0.0])

    dir_world = target - cam_origin
    dir_world /= np.linalg.norm(dir_world)
    d_opt = R_OPTICAL_TO_LINK.T @ (R_world_cam.T @ dir_world)
    uv = K_REAL @ d_opt
    u, v = uv[0]/uv[2], uv[1]/uv[2]

    d2 = optical_to_world(pixel_to_ray(u, v, K_REAL), R_world_cam)
    p = intersect_ground(cam_origin, d2)

    assert p is not None
    assert p == pytest.approx((target[0], target[1]), abs=1e-6)