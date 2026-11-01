"""一键启动：清理 -> Gazebo(含 ekf_local) -> Nav2。

设计要点见 README §29。
Gazebo GUI 默认保留，不再自动关闭；需要省 CPU 时手动 pkill -x gzclient。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')

    # ============================================================
    # 阶段 0：清理（残留进程 + DDS 共享内存）
    # ============================================================
    cleanup = ExecuteProcess(
        cmd=['bash', '-c', ' '.join([
            'pkill -9 -x gzserver;',
            'pkill -9 -x gzclient;',
            'pkill -9 -f "[n]av2";',
            'pkill -9 -f "[e]kf_node";',
            'pkill -9 -x collision_monitor;',
            'pkill -9 -x robot_state_publisher;',
            'pkill -9 -f "[r]ed_target_detector";',
            'pkill -9 -f "[g]o_to_target";',
            # 等旧进程真正退出，再清它们留下的共享内存段
            'sleep 3;',
            'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*;',
            'sleep 1;',
            'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*;',
            'echo "[bringup] 清理完成，残留 fastrtps 段:'
            ' $(ls /dev/shm 2>/dev/null | grep -c fastrtps)"',
        ])],
        output='screen',
    )

    # ============================================================
    # 阶段 1：Gazebo（复用 gazebo.launch.py，其中已包含 ekf_local）
    # ============================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
    )

    # ============================================================
    # 阶段 2：Nav2（复用 nav2_bringup 的 launch）
    # ============================================================
    # 用路径替换而不是写死 $HOME/ros2/maps：换机器、换目录都不用改
    map_yaml = PathJoinSubstitution([
        FindPackageShare('my_robot_description'), 'maps', 'my_map.yaml'])
    nav2_params = PathJoinSubstitution([
        FindPackageShare('my_robot_description'), 'config', 'nav2_params.yaml'])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': 'true',
            'use_composition': 'True',
        }.items(),
    )

    return LaunchDescription([
        # ★ 关键：禁用 Fast DDS 共享内存传输，只用 UDPv4。
        # 否则 /dev/shm 里的 fastrtps 段会抢 fastrtps_port74xx，
        # 导致 gzserver 的 ROS 插件初始化失败（/odom /scan 全都没有）。
        # 必须放在列表最前面，才能影响后面所有子进程。
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),

        cleanup,
        # 清理自身含 sleep 3 + 1，约 5 秒后起 Gazebo
        TimerAction(period=5.0, actions=[gazebo]),
        # 保留 GUI 时 spawn 更慢，Nav2 延到 15 秒
        TimerAction(period=15.0, actions=[nav2]),
    ])