#!/usr/bin/env python3
"""启动 ros2_control 差速底盘 + 传感器 + 安全链 + 局部 EKF（不用 Gazebo）。

启动顺序（每一步都有依赖，不是随便排的）
----------------------------------------
  0  cmd_vel_adapter      私有话题桥接：/cmd_vel_final <-> diff_drive_controller
  1  robot_state_publisher  base_link -> 轮子/激光/IMU 的静态 TF
  2  ros2_control_node     controller_manager，加载 DiffDriveHardware + ImuSensor
  3  joint_state_broadcaster  关节状态（position/velocity/effort）
  4  diff_drive_controller    差速运动学 -> /diff_drive_controller/odom
  5  imu_sensor_broadcaster   SensorInterface -> /imu_sensor_broadcaster/imu
  6  laser_scan_simulator     ray-casting -> /scan
  7  cmd_vel_watchdog         /cmd_vel -> /cmd_vel_safe（0.5 s 无指令就停车）
  8  collision_monitor        /cmd_vel_safe -> /cmd_vel_final（降速 + 急停）
  9  ekf_local                /odom + IMU -> odom -> base_link + /odometry/local

安全链在话题上串联，与底盘驱动完全解耦：
  上游 /cmd_vel  ──watchdog──▶ /cmd_vel_safe ──collision_monitor──▶ /cmd_vel_final ──▶ 适配器
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('my_robot_control')

    xacro_file = os.path.join(pkg_share, 'urdf', 'diff_robot.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')
    collision_file = os.path.join(pkg_share, 'config', 'collision_monitor.yaml')
    ekf_local_file = os.path.join(pkg_share, 'config', 'ekf_local.yaml')

    # ── 可配置参数 ──
    motor_mode_arg = DeclareLaunchArgument(
        'motor_mode', default_value='velocity',
        description='底盘驱动模式：velocity（速度闭环）或 effort（纯力矩）')

    motor_mode = LaunchConfiguration('motor_mode')

    # xacro 生成 URDF，把 motor_mode 传进去
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' motor_mode:=', motor_mode]),
        value_type=str
    )

    # ── 0. 话题适配器 ──
    #    入口必须是安全链的出口 /cmd_vel_final。
    #    如果写成 /cmd_vel，就会绕过 watchdog 和 collision_monitor
    #    （表面上看能动，但安全链形同虚设 —— 这类"接错一环"最难发现）
    adapter_input = '/cmd_vel_final'
    cmd_vel_adapter = Node(
        package='my_robot_tasks',
        executable='cmd_vel_adapter',
        name='cmd_vel_adapter',
        output='screen',
        parameters=[{'input_topic': adapter_input}]
    )

    # ── 1. 机器人模型与静态 TF ──
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ── 2. controller_manager（加载两个硬件插件）──
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
        ],
        output='screen'
    )

    # ── 3~5. 三个广播器/控制器 ──
    def spawner(name, *extra):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager', *extra],
            output='screen'
        )

    # ── 6. 激光模拟器 ──
    laser_scan_simulator = Node(
        package='my_robot_control',
        executable='laser_scan_simulator',
        name='laser_scan_simulator',
        output='screen'
    )

    # ── 7. 速度指令看门狗（/cmd_vel -> /cmd_vel_safe）──
    cmd_vel_watchdog = Node(
        package='my_first_pkg',
        executable='cmd_vel_watchdog',
        name='cmd_vel_watchdog',
        output='screen',
        parameters=[{'timeout_sec': 0.5}]
    )

    # ── 8. 碰撞监控（/cmd_vel_safe -> /cmd_vel_final），需要 lifecycle 管理 ──
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_file]
    )
    collision_monitor_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['collision_monitor'],
        }]
    )

    # ── 9. 第 1 级 EKF（odom 系融合，接管 odom -> base_link）──
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_local_file],
        remappings=[('odometry/filtered', '/odometry/local')]
    )

    return LaunchDescription([
        motor_mode_arg,

        # ① 基础设施（立刻起）
        cmd_vel_adapter,
        robot_state_publisher,
        control_node,

        # ② 控制器（controller_manager 起来之后）
        TimerAction(period=2.0, actions=[
            spawner('joint_state_broadcaster'),
            spawner('diff_drive_controller'),
            spawner('imu_sensor_broadcaster'),
        ]),

        # ②b 预加载备用控制器（--inactive：加载并配置，但不激活）
        #     必须先加载，否则 `ros2 control switch_controllers` 会因默认的
        #     BEST_EFFORT 严格度「静默跳过」不存在的控制器，打印 Successfully
        #     却什么都没做 —— 这是最容易骗过自己的假成功。
        TimerAction(period=3.0, actions=[
            spawner('manual_velocity_controller', '--inactive'),
            spawner('manual_effort_controller', '--inactive'),
        ]),

        # ③ 传感器与安全链（/scan 和 TF 就绪之后）
        TimerAction(period=5.0, actions=[
            laser_scan_simulator,
            cmd_vel_watchdog,
            collision_monitor,
            collision_monitor_lifecycle,
        ]),

        # ④ EKF 最后起（要 /odom 和 IMU 都有数据）
        TimerAction(period=8.0, actions=[ekf_local]),
    ])
