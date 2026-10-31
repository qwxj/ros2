import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 启动 Gazebo（gzserver + 客户端 GUI）
    world_path = os.path.join(pkg_share, 'worlds', 'my_world.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    return LaunchDescription([
        gazebo,

        # 把 URDF 变成 /robot_description 和 TF
        # use_sim_time=True：让 TF 时间戳跟 Gazebo 的仿真时钟走
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
            output='screen',
        ),

        # 把机器人模型生成到 Gazebo 世界里
        # -z 0.5：抬高半米生成，这样能看到它自由落体落地
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'my_car',
                       '-z', '0.5'],
            output='screen',
        ),

        Node(
            package='my_first_pkg',
            executable='cmd_vel_watchdog',
            parameters=[{'timeout_sec': 0.5}],
            output='screen',
        ),
    ])