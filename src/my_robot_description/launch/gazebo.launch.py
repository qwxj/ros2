import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 启动 Gazebo（gzserver + 客户端 GUI）
    world_path = os.path.join(pkg_share, 'worlds', 'my_world.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # 物理安全层的参数文件
    collision_params = os.path.join(pkg_share, 'config', 'collision_monitor.yaml')

    return LaunchDescription([
        gazebo,

        # 把 URDF 变成 /robot_description 和 TF
        # use_sim_time=True：让 TF 时间戳跟 Gazebo 的仿真时钟走
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
            output='screen',
        ),

        # 把机器人模型生成到 Gazebo 世界里
        # -z 0.5：抬高半米生成，这样能看到它自由落体落地
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'my_car',
                       '-z', '0.5'],
            output='screen',
        ),

        Node(
            package='my_first_pkg',
            executable='cmd_vel_watchdog',
            parameters=[{'timeout_sec': 0.5}],
            output='screen',
        ),

                # ============================================================
        # 物理安全层：前方有障碍时降速 / 急停
        # 它是 Lifecycle 节点，单独启动会停在 unconfigured，
        # 必须由 lifecycle_manager 来 configure + activate
        # ============================================================
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            parameters=[collision_params, {'use_sim_time': True}],
            output='screen',
        ),

        # 延迟 10 秒再启动生命周期管理器：
        # 等 Gazebo 把车 spawn 出来、TF 和 /scan 就绪，否则 activate 会失败
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_safety',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': ['collision_monitor'],
                    }],
                    output='screen',
                ),
            ],
        ),
    ])