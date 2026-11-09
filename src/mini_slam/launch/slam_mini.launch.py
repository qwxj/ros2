"""启动 SLAM 节点（可选同时打开 RViz）。

用法：
    ros2 launch mini_slam slam_mini.launch.py
    ros2 launch mini_slam slam_mini.launch.py robot:=diff_small
    ros2 launch mini_slam slam_mini.launch.py robot:=diff_small rviz:=false   # 无图形界面时

参数加载顺序：
    config/slam_mini.yaml          （通用默认）
    config/robots/<robot>.yaml     （机型覆盖，存在才加载）
后加载的同名字段会覆盖前面的。

RViz 的配置在 config/rviz/mini_slam.rviz：
    Fixed Frame = map
    Map        /map        （Transient Local，晚加入也收得到）
    LaserScan  /scan       （Best Effort）
    RobotModel /robot_description
    AMCL Pose  /amcl_pose
    TF         全开，能看到 map -> odom -> base_link -> laser_link
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory('mini_slam')
    config_dir = os.path.join(share_dir, 'config')

    robot = LaunchConfiguration('robot').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'

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

    actions = [
        Node(
            package='mini_slam',
            executable='slam_mini_node',
            name='slam_mini',
            output='screen',
            parameters=params,
        )
    ]

    if rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config_dir, 'rviz', 'mini_slam.rviz')],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='default',
            description='机型名，对应 config/robots/<robot>.yaml'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否同时打开 RViz（无图形界面时传 false）'),
        OpaqueFunction(function=launch_setup),
    ])
