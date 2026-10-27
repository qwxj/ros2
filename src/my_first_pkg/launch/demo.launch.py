
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('my_first_pkg'),
        'config',
        'parameter_demo.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_first_pkg',
            executable='my_publisher',
            name='demo_publisher',
            output='screen'
        ),
        Node(
            package='my_first_pkg',
            executable='my_subscriber',
            name='demo_subscriber',
            output='screen'
        ),
        Node(
            package='my_first_pkg',
            executable='parameter_demo',
            name='parameter_demo',
            parameters=[config_file],
            output='screen'
        ),
    ])

