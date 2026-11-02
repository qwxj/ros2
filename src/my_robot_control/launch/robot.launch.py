
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # 获取当前功能包的安装目录
    pkg_share = get_package_share_directory('my_robot_control')

    # URDF/Xacro 文件路径
    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'diff_robot.urdf.xacro'
    )

    # 控制器参数文件路径
    controllers_file = os.path.join(
        pkg_share,
        'config',
        'controllers.yaml'
    )

    # 执行 xacro，将机器人模型转换成 URDF 字符串
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 0. 标准话题适配器：把 ros2_control 的私有话题桥接成 /cmd_vel 和 /odom
    #    diff_drive_controller 实际用的是 ~/cmd_vel_unstamped 和 ~/odom（带节点名前缀），
    #    Nav2 / teleop 用的是 /cmd_vel 和 /odom，必须桥接否则车不动。
    cmd_vel_adapter = Node(
        package='my_robot_tasks',
        executable='cmd_vel_adapter',
        name='cmd_vel_adapter',
        output='screen'
    )

    # 1. 发布机器人模型和 TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description}
        ]
    )

    # 2. 启动 ros2_control 和硬件插件
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen'
    )

    # 3. 加载关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # 4. 加载差速控制器
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # 5. 激光雷达模拟器（用 map/世界里的线段做 ray-casting，输出 /scan）
    #    没有它就没有 /scan，AMCL 无法定位，Nav2 全线起不来。
    laser_scan_simulator = Node(
        package='my_robot_control',
        executable='laser_scan_simulator',
        name='laser_scan_simulator',
        output='screen'
    )

    return LaunchDescription([
        cmd_vel_adapter,
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        laser_scan_simulator
    ])
