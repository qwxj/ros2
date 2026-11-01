# -*- coding: utf-8 -*-
"""纯几何计算，不依赖 ROS —— 可以被 pytest 直接测试。

这些函数是视觉导航的地基：
    像素 --pixel_to_ray--> 光学系方向
         --to_world------> 世界系方向
         --intersect_ground--> 地面上的位置
"""
import numpy as np

# 光学坐标系（ROS 图像约定：x 右 / y 下 / z 前）
# -> 车体坐标系（x 前 / y 左 / z 上）
R_OPTICAL_TO_LINK = np.array([[0.0, 0.0, 1.0],
                              [-1.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0]])


def quat_xyzw_to_matrix(x, y, z, w):
    """四元数 (x,y,z,w) -> 3x3 旋转矩阵。

    只接收四个浮点数（而不是 ROS 的 Quaternion 对象），
    这样测试里可以随便造数据。
    """
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def pixel_to_ray(u, v, K):
    """像素 (u,v) -> 单位方向向量（相机光学坐标系）。

    K 是 3x3 内参矩阵。用 solve 而不是求逆，数值更稳。
    """
    d = np.linalg.solve(np.asarray(K, dtype=float), np.array([u, v, 1.0]))
    return d / np.linalg.norm(d)


def optical_to_world(d_opt, R_world_cam):
    """光学系方向 -> 世界系方向。

    R_world_cam 是相机在世界系里的旋转（来自 TF）。
    """
    return R_world_cam @ (R_OPTICAL_TO_LINK @ np.asarray(d_opt, dtype=float))


def intersect_ground(origin, direction):
    """从 origin 沿 direction 射出去，与地面 z=0 求交。

    返回 (x, y)；射线朝上/平行地面/交点在背后时返回 None。
    """
    origin = np.asarray(origin, dtype=float)
    direction = np.asarray(direction, dtype=float)
    if direction[2] >= -1e-9:
        return None
    s = -origin[2] / direction[2]
    if s <= 0.0:
        return None
    return (float(origin[0] + s * direction[0]),
            float(origin[1] + s * direction[1]))