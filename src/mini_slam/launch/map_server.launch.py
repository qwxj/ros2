"""只启动地图服务（map_server + 自动激活），不启动定位算法。

用法：
    ros2 launch mini_slam map_server.launch.py                  # 包内 demo_map
    ros2 launch mini_slam map_server.launch.py map:=demo_map    # 包内其它图
    ros2 launch mini_slam map_server.launch.py map:=/abs/x.yaml # 任意绝对路径

启动后：
    /map                                       （transient_local，晚加入也收得到）
    /map_server/...                            （生命周期服务）

map:= 支持三种写法：
    ① 绝对路径   /home/wxj/mini_map.yaml
    ② 包内名字   demo_map          → <share>/mini_slam/maps/demo_map.yaml
    ③ 空字符串   ''                → 本 launch 直接报错（必须给图）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def resolve_map(spec):
    """把 map:= 的写法解析成真实的 yaml 绝对路径。"""
    spec = spec.strip()
    if not spec:
        raise RuntimeError('map:= 不能为空（本 launch 就是用来起地图服务的）')

    if os.path.isabs(spec):
        if not os.path.exists(spec):
            raise RuntimeError('地图文件不存在: {}'.format(spec))
        return spec

    maps_dir = os.path.join(get_package_share_directory('mini_slam'), 'maps')
    name = spec if spec.endswith('.yaml') else spec + '.yaml'
    path = os.path.join(maps_dir, name)
    if not os.path.exists(path):
        available = sorted(
            f[:-5] for f in os.listdir(maps_dir) if f.endswith('.yaml')
        ) if os.path.isdir(maps_dir) else []
        raise RuntimeError(
            '包内找不到地图: {}\n包内可用: {}'.format(path, available))
    return path


def launch_setup(context, *args, **kwargs):
    map_yaml = resolve_map(LaunchConfiguration('map').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    return [
        LogInfo(msg='地图文件: {}'.format(map_yaml)),
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 0.0,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='demo_map',
            description='地图：绝对路径 / 包内名字（maps/<name>.yaml）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
