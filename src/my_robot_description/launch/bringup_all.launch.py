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
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')

    # ── 可配置参数（等价于命令行 flag）──
    headless = LaunchConfiguration('headless')

    # ══════════════════════════════════════════════════════════
    # 阶段 0：清理（残留进程 + DDS 共享内存）
    # ══════════════════════════════════════════════════════════
    cleanup = ExecuteProcess(
        cmd=['bash', '-c', ' '.join([
            'pkill -9 -f "[g]zserver";',
            'pkill -9 -f "[g]zclient";',
            'pkill -9 -f "[n]av2";',
            'pkill -9 -f "[c]ollision_monitor";',
            'pkill -9 -f "[r]obot_state_publisher";',
            'pkill -9 -f "[r]ed_target_detector";',
            'pkill -9 -f "[g]o_to_target";',
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
    # 阶段 1.5：15 秒后杀掉 gzclient（省 48% CPU）
    # ══════════════════════════════════════════════════════════
    kill_gui = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'pkill -f gzclient; '
                 'echo "[bringup] 已关闭 gzclient（headless 模式）"'],
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
        kill_gui,
    ])