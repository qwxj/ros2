from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('my_first_pkg'),
        'config',
        'params.yaml'
    )

    publisher_node = Node(
        package='my_first_pkg',
        executable='my_publisher',
        name='my_publisher',
        output='screen'
    )

    status_node = Node(
        package='my_first_pkg',
        executable='status_node',
        name='status_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        publisher_node,
        status_node
    ])