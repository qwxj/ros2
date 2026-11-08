"""启动 AMCL 定位（含地图服务 + 自动激活）。

用法：
    ros2 launch mini_slam amcl_mini.launch.py
    ros2 launch mini_slam amcl_mini.launch.py map:=demo_map
    ros2 launch mini_slam amcl_mini.launch.py map:=/home/wxj/mini_map.yaml
    ros2 launch mini_slam amcl_mini.launch.py map:=none       # 用外部的 /map
    ros2 launch mini_slam amcl_mini.launch.py robot:=diff_small

启动内容：
    map 非空时：  map_server + lifecycle_manager（自动 config+activate）+ amcl_mini
    map 为空时：  只起 amcl_mini（假定别处已有 /map 在发布）

map:= 支持三种写法：
    ① 绝对路径   /home/wxj/mini_map.yaml
    ② 包内名字   demo_map          → <share>/mini_slam/maps/demo_map.yaml
    ③ none       none              → 不起地图服务，直接等外部的 /map
       （不用空字符串：ROS 2 launch 不接受 map:= 后面没有值，
          bash 也会把 '' 吃掉，只剩 map:= 会报 malformed launch argument）

注意：amcl_mini 会发布 map->odom，不能和 slam_mini_node 同时运行。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def resolve_map(spec):
    """把 map:= 解析成 yaml 绝对路径；空字符串返回 None（不起地图服务）。"""
    spec = spec.strip()
    # 注意：不能用空字符串来表示"不要地图服务"。
    #   `ros2 launch ... map:=''` 在 bash 里会变成 `map:=`，
    #   ROS 2 launch 直接拒绝：malformed launch argument 'map:='
    #   所以用哨兵值 none（off 也接受，容错）
    if spec.lower() in ('', 'none', 'off'):
        return None

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
            '包内找不到地图: {}\n'
            '包内可用: {}\n'
            '（想不起地图服务用 map:=none）'.format(path, available))
    return path


def launch_setup(context, *args, **kwargs):
    config_dir = os.path.join(get_package_share_directory('mini_slam'), 'config')
    robot = LaunchConfiguration('robot').perform(context)
    map_yaml = resolve_map(LaunchConfiguration('map').perform(context))
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

    # ---------- 地图服务（map 为空则跳过）----------
    if map_yaml:
        actions.append(LogInfo(msg='地图文件: {}'.format(map_yaml)))
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
                'bond_timeout': 0.0,
            }],
        ))
    else:
        actions.append(LogInfo(
            msg='map:=none → 不启动地图服务，等待外部发布的 /map'))

    # ---------- AMCL ----------
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
            'map', default_value='demo_map',
            description='地图：绝对路径 / 包内名字 / none（不起地图服务）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
