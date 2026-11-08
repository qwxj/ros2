"""启动 AMCL 定位（含地图服务）。

用法：
    ros2 launch mini_slam amcl_mini.launch.py
    ros2 launch mini_slam amcl_mini.launch.py map:=/home/wxj/mini_map.yaml
    ros2 launch mini_slam amcl_mini.launch.py robot:=diff_small
    ros2 launch mini_slam amcl_mini.launch.py map:=''      # 不起地图服务

启动内容：
    ① map_server            （nav2_map_server，生命周期节点）
    ② lifecycle_manager_map （自动 configure + activate ①）
    ③ amcl_mini             （本包的定位节点）

注意：amcl_mini 会发布 map->odom，不能和 slam_mini_node 同时运行。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def launch_setup(context, *args, **kwargs):
    config_dir = os.path.join(get_package_share_directory('mini_slam'), 'config')
    robot = LaunchConfiguration('robot').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    # ---------- 参数：通用 + 机型覆盖 ----------
    params = [os.path.join(config_dir, 'amcl_mini.yaml')]
    robot_yaml = os.path.join(config_dir, 'robots', robot + '.yaml')
    if os.path.exists(robot_yaml):
        params.append(robot_yaml)
    else:
        available = sorted(
            f[:-5] for f in os.listdir(os.path.join(config_dir, 'robots'))
            if f.endswith('.yaml'))
        raise RuntimeError(
            '找不到机型配置: {}\n可用的机型: {}'.format(robot_yaml, available))

    actions = []

    # ---------- ① + ② 地图服务（可选）----------
    if map_yaml:
        if not os.path.exists(map_yaml):
            raise RuntimeError('地图文件不存在: {}'.format(map_yaml))

        actions.append(LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',        # Humble 的 LifecycleNode 必须显式给
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
            }],
        ))
        actions.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 0.0,       # 不用 bond，省掉一个失败点
            }],
        ))

    # ---------- ③ AMCL ----------
    actions.append(Node(
        package='mini_slam',
        executable='amcl_mini_node',
        name='amcl_mini',
        output='screen',
        parameters=params,
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='default',
            description='机型名，对应 config/robots/<robot>.yaml'),
        DeclareLaunchArgument(
            'map', default_value='/home/wxj/mini_map.yaml',
            description='地图 yaml 路径；传空字符串则不启动地图服务'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时间'),
        OpaqueFunction(function=launch_setup),
    ])
