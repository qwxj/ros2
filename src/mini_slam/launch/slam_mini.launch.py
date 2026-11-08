"""启动 SLAM 节点。

用法：
    ros2 launch mini_slam slam_mini.launch.py
    ros2 launch mini_slam slam_mini.launch.py robot:=diff_small

参数加载顺序：
    config/slam_mini.yaml          （通用默认）
    config/robots/<robot>.yaml     （机型覆盖，存在才加载）
后加载的同名字段会覆盖前面的。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_dir = os.path.join(get_package_share_directory('mini_slam'), 'config')
    robot = LaunchConfiguration('robot').perform(context)

    params = [os.path.join(config_dir, 'slam_mini.yaml')]

    robot_yaml = os.path.join(config_dir, 'robots', robot + '.yaml')
    if os.path.exists(robot_yaml):
        params.append(robot_yaml)
    else:
        available = sorted(
            f[:-5] for f in os.listdir(os.path.join(config_dir, 'robots'))
            if f.endswith('.yaml'))
        raise RuntimeError(
            '找不到机型配置: {}\n可用的机型: {}'.format(robot_yaml, available))

    return [
        Node(
            package='mini_slam',
            executable='slam_mini_node',
            name='slam_mini',
            output='screen',
            parameters=params,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='default',
            description='机型名，对应 config/robots/<robot>.yaml'),
        OpaqueFunction(function=launch_setup),
    ])
