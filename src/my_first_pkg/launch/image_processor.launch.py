import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('my_first_pkg'),
        'config', 'image_processor.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_first_pkg',
            executable='image_processor',
            name='image_processor',      # ← 必须和 YAML 里的键名一致
            parameters=[params],
            output='screen',
        ),
    ])