import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_hardware')

    with open(os.path.join(pkg, 'urdf', 'test.urdf'), 'r') as f:
        robot_description = f.read()

    controllers_yaml = os.path.join(pkg, 'config', 'controllers.yaml')

    # 发布 TF，RViz 里能看到关节在动
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ★ 硬件插件 + 控制器都在这个进程里
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
    )

    # ★ spawner 会自动等 controller_manager 起来
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
    ])