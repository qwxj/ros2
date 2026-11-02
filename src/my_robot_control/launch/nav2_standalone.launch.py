#!/usr/bin/env python3
"""启动地图 + AMCL + 第 2 级 EKF + Nav2（不用 Gazebo）。

与 robot.launch.py 配合使用，必须先起 robot.launch.py：
  robot.launch.py    提供 /scan、/odom、/odometry/local、odom -> base_link
  nav2_standalone.py 提供 /map、map -> odom、以及整条 Nav2 规划控制链

两级 EKF 的分工：
  ekf_local  （robot.launch.py 里）odom 系内融合轮式 + IMU   -> odom -> base_link
  ekf_global （本文件）           map  系内融合 AMCL + 局部速度 -> map  -> odom
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = get_package_share_directory('my_robot_control')
    ekf_global_file = os.path.join(pkg_share, 'config', 'ekf_global.yaml')

    map_yaml = os.path.join(pkg_share, 'maps', 'map.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params_standalone.yaml')

    # ── 1. 地图服务器 ──
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False, 'yaml_filename': map_yaml}]
    )

    # ── 2. AMCL（map 系绝对定位）──
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': False}, params_file]
    )

    # ── 3. 第 2 级 EKF（接管 map -> odom；AMCL 的 tf_broadcast 已关）──
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_global_file],
        remappings=[('odometry/filtered', '/odometry/global')]
    )

    # ── 4. map_server + amcl 的生命周期管理 ──
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }]
    )

    # ── 5. Nav2 规划控制链 ──
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items()
    )

    return LaunchDescription([
        map_server,
        amcl,
        localization_lifecycle,

        # AMCL 有 /amcl_pose 之后才起 ekf_global
        TimerAction(period=6.0, actions=[ekf_global]),

        # Nav2 最后起（依赖 map -> odom 存在）
        TimerAction(period=9.0, actions=[navigation]),
    ])
