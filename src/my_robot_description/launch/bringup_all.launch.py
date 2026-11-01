"""一键启动：清理 → Gazebo → 自动关闭 gzclient。

设计要点见 README §29。这里先只做阶段 0 和阶段 1。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')

    # ── 可配置参数（等价于命令行 flag）──
    headless = LaunchConfiguration('headless')

    # ══════════════════════════════════════════════════════════
    # 阶段 0：清理（残留进程 + DDS 共享内存）
    # ══════════════════════════════════════════════════════════
    cleanup = ExecuteProcess(
        cmd=['bash', '-c', ' '.join([
            'pkill -9 -x gzserver;',
            'pkill -9 -x gzclient;',
            'pkill -9 -f "[n]av2";',          # nav2 是一类进程（ros2 launch ... nav2_bringup），没有单一进程名
            'pkill -9 -x collision_monitor;',
            'pkill -9 -x robot_state_publisher;',
            'pkill -9 -f "[r]ed_target_detector";',   # 这个实际进程名是 python3，只能用 -f
            'pkill -9 -f "[g]o_to_target";',          # 同上
            'sleep 2;',
            'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*;',
            'echo "[bringup] 清理完成，残留 fastrtps 段:'
            ' $(ls /dev/shm 2>/dev/null | grep -c fastrtps)"',
        ])],
        output='screen',
    )

    # ══════════════════════════════════════════════════════════
    # 阶段 1：Gazebo（复用已有的 gazebo.launch.py）
    # ══════════════════════════════════════════════════════════
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
    )
    # ══════════════════════════════════════════════════════════
    # 阶段 2：Nav2（复用 nav2_bringup 的 launch）
    # ══════════════════════════════════════════════════════════
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
            'use_composition': 'False',
        }.items(),
    )

    # ══════════════════════════════════════════════════════════
    # 阶段 1.5：15 秒后杀掉 gzclient（省 48% CPU）
    # ══════════════════════════════════════════════════════════
    kill_gui = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'pkill -9 -x gzclient && '
                 'echo "[bringup] 已关闭 Gazebo GUI（headless）" '
                 '|| echo "[bringup] 警告：Gazebo GUI 没被杀掉"'],
            output='screen',
        )],
        condition=IfCondition(headless),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='true=15 秒后关闭 Gazebo GUI 省 CPU；false=保留 GUI'),
        cleanup,
        # ★ 关键：等清理跑完（4 秒）再起 Gazebo
        TimerAction(period=4.0, actions=[gazebo]),
        # 等 Gazebo spawn 完车、/scan 有数据，再起 Nav2
        TimerAction(period=12.0, actions=[nav2]),
        kill_gui,
    ])