
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 地图文件路径
    map_yaml = PathJoinSubstitution([
        FindPackageShare('my_robot_control'),
        'maps',
        'map.yaml'
    ])

    # 1. 地图服务器
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': map_yaml
        }]
    )

    # 2. 地图服务器生命周期管理
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # 3. 【已移除】静态 map -> odom
    #    静态 TF 等于"假设机器人永远在原点"，一旦走偏就再也纠不回来。
    #    现在改由 AMCL 根据 /scan 与地图匹配来发布 map -> odom（见下方）。

    # 4. Nav2 导航服务器
    nav2_launch = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    ])

    params_file = PathJoinSubstitution([
        FindPackageShare('my_robot_control'),
        'config',
        'nav2_params_standalone.yaml'
    ])

    # 5. AMCL：用 /scan + 地图做粒子滤波定位，发布 map -> odom
    #    参数从同一个 nav2_params_standalone.yaml 的 amcl: 段读取
    #    （base_frame_id: base_link、scan_topic: /scan、set_initial_pose: true）
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            params_file,
        ]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info'
        }.items()
    )

    return LaunchDescription([
        map_server,
        map_lifecycle_manager,
        amcl,
        navigation
    ])
