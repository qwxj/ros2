# 指定 CMake 最低版本要求
cmake_minimum_required(VERSION 3.8)

# 定义 ROS 2 功能包名称
# 该名称需要与 package.xml 中的包名一致
project(my_first_pkg)

# 如果使用 GCC 或 Clang 编译器，则启用常见警告
# -Wall      ：启用大部分常见警告
# -Wextra    ：启用额外警告
# -Wpedantic ：提示不符合标准 C++ 的写法
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# 一、查找编译所需的依赖包
# ============================================================

# ROS 2 的 CMake 构建工具
find_package(ament_cmake REQUIRED)

# ROS 2 C++ 客户端库：Node、Publisher、Subscriber、Service 等
find_package(rclcpp REQUIRED)

# ROS 2 标准消息：例如 std_msgs/msg/String
find_package(std_msgs REQUIRED)

# ROS 2 示例接口：例如 AddTwoInts 服务、Fibonacci Action
find_package(example_interfaces REQUIRED)

# ROS 2 Action 客户端与服务端功能
find_package(rclcpp_action REQUIRED)

# 自定义接口包：包含自己定义的 .msg 和 .srv
find_package(my_robot_interfaces REQUIRED)

# TF2 坐标变换相关基础功能
find_package(tf2 REQUIRED)

# ROS 2 中 TF2 的广播器、监听器等功能
find_package(tf2_ros REQUIRED)

# 几何消息：例如 TransformStamped、Point、Pose
find_package(geometry_msgs REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)

# 查找生命周期相关文件
find_package(rclcpp_lifecycle REQUIRED)

# 组件
find_package(rclcpp_components REQUIRED)

# ============================================================
# 二、Topic：基础发布者与订阅者
# ============================================================

# 创建发布者可执行程序
# 源文件：src/my_publisher.cpp
# 生成的程序名：my_publisher
add_executable(my_publisher src/my_publisher.cpp)

# 声明该程序依赖 rclcpp 和 std_msgs
ament_target_dependencies(my_publisher rclcpp std_msgs)


# 创建 QoS 测试发布者
# 用于学习可靠性、历史记录等 QoS 配置
add_executable(qos_publisher src/qos_publisher.cpp)
ament_target_dependencies(qos_publisher rclcpp std_msgs)


# 创建基础订阅者
# 订阅 my_publisher 发布的标准消息
add_executable(my_subscriber src/my_subscriber.cpp)
ament_target_dependencies(my_subscriber rclcpp std_msgs)


# ============================================================
# 三、Service：服务端与客户端
# ============================================================

# 创建 AddTwoInts 服务端
add_executable(add_server src/add_server.cpp)
ament_target_dependencies(add_server rclcpp example_interfaces)

# 创建 AddTwoInts 客户端
add_executable(add_client src/add_client.cpp)
ament_target_dependencies(add_client rclcpp example_interfaces)


# ============================================================
# 四、Action：动作服务端与客户端
# ============================================================

# 创建 Action 服务端
# 使用 Fibonacci Action，支持反馈和最终结果
add_executable(action_server src/action_server.cpp)

ament_target_dependencies(
  action_server
  rclcpp
  rclcpp_action
  example_interfaces
)

# 创建 Action 客户端
add_executable(action_client src/action_client.cpp)

ament_target_dependencies(
  action_client
  rclcpp
  rclcpp_action
  example_interfaces
)


# ============================================================
# 五、参数管理
# ============================================================

# 创建参数演示节点
# 用于学习参数声明、查询、修改以及 YAML 参数文件
add_executable(parameter_demo src/parameter_demo.cpp)

ament_target_dependencies(
  parameter_demo
  rclcpp
)


# ============================================================
# 六、Executor 与回调并发
# ============================================================

# 创建执行器演示节点
# 用于学习定时器、回调执行、单线程与多线程执行器
add_executable(executor_demo src/executor_demo.cpp)

ament_target_dependencies(
  executor_demo
  rclcpp
)


# ============================================================
# 七、自定义 Service：设备状态查询
# ============================================================

# 创建自定义设备状态服务端
# 使用 my_robot_interfaces/srv/GetDeviceStatus
add_executable(
  get_device_status_server
  src/get_device_status_server.cpp
)

ament_target_dependencies(
  get_device_status_server
  rclcpp
  my_robot_interfaces
)

# 创建自定义设备状态客户端
# 向服务端发送设备名称，并接收查询结果
add_executable(
  get_device_status_client
  src/get_device_status_client.cpp
)

ament_target_dependencies(
  get_device_status_client
  rclcpp
  my_robot_interfaces
)


# ============================================================
# 八、自定义 Topic：设备状态消息
# ============================================================

# 创建自定义消息发布者
# 使用 my_robot_interfaces/msg/DeviceStatus
# 定时发布设备名称、在线状态和温度
add_executable(
  device_status_publisher
  src/device_status_publisher.cpp
)

ament_target_dependencies(
  device_status_publisher
  rclcpp
  my_robot_interfaces
)

# 创建自定义消息订阅者
# 接收设备状态消息，并在回调函数中处理数据
add_executable(
  device_status_subscriber
  src/device_status_subscriber.cpp
)

ament_target_dependencies(
  device_status_subscriber
  rclcpp
  my_robot_interfaces
)


# ============================================================
# 九、TF2 坐标变换
# ============================================================

# 下一步添加 TF2 广播器时，取消下面这段注释。
# 前提是已经创建 src/tf2_broadcaster_demo.cpp。

add_executable(
  tf2_broadcaster_demo
  src/tf2_broadcaster_demo.cpp
)

# 声明 TF2 节点依赖：
# rclcpp       ：ROS 2 C++ 节点功能
# tf2          ：坐标变换基础库
# tf2_ros      ：坐标变换广播器等 ROS 2 接口
# geometry_msgs：TransformStamped 等几何消息类型
ament_target_dependencies(
  tf2_broadcaster_demo
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)


# 声明监听器，转换坐标
add_executable(
  tf2_listener_demo
  src/tf2_listener_demo.cpp
)

ament_target_dependencies(
  tf2_listener_demo
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)


# 障碍坐标信息
add_executable(
  tf2_point_transform_demo
  src/tf2_point_transform_demo.cpp
)

ament_target_dependencies(
  tf2_point_transform_demo
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# odom动态转换
add_executable(
  tf2_odom_broadcaster_demo
  src/tf2_odom_broadcaster_demo.cpp
)

ament_target_dependencies(
  tf2_odom_broadcaster_demo
  rclcpp
  geometry_msgs
  tf2_ros
)

# 四元坐标
add_executable(
  quaternion_demo
  src/quaternion_demo.cpp
)

ament_target_dependencies(
  quaternion_demo
  rclcpp
  geometry_msgs
  tf2_ros
)

# 模拟障碍物信息
add_executable(laser_point_publisher
  src/laser_point_publisher.cpp
)

ament_target_dependencies(laser_point_publisher
  rclcpp
  geometry_msgs
)

# 坐标转换节点
add_executable(laser_point_transformer
  src/laser_point_transformer.cpp
)

ament_target_dependencies(laser_point_transformer
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 单线程调度器
add_executable(executor_single_demo src/executor_single_demo.cpp)
ament_target_dependencies(executor_single_demo rclcpp)
# 多线程调度器
add_executable(executor_multi_demo src/executor_multi_demo.cpp)
ament_target_dependencies(executor_multi_demo rclcpp)


# 生命周期管理
add_executable(lifecycle_demo src/lifecycle_demo.cpp)

ament_target_dependencies(lifecycle_demo
  rclcpp
  rclcpp_lifecycle
)


# 单纯的节点
add_executable(status_node src/status_node.cpp)

ament_target_dependencies(status_node
  rclcpp
)


## 组件学习
add_library(status_component SHARED
  src/status_component.cpp
)

ament_target_dependencies(status_component
  rclcpp
  rclcpp_components
)

rclcpp_components_register_nodes(status_component
  "StatusComponent"
)

install(TARGETS
  status_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## 一个组件，俩节点运行
add_library(topic_components SHARED
  src/topic_components.cpp
)

ament_target_dependencies(topic_components
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_nodes(topic_components
  "DemoPublisher"
  "DemoSubscriber"
)

install(TARGETS topic_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# ============================================================
# 十、安装所有可执行程序
# ============================================================

# colcon build 不仅要编译程序，还需要把程序安装到包的安装目录。
# 安装后才能通过 ros2 run my_first_pkg 程序名运行。
install(
  TARGETS
    my_publisher
    qos_publisher
    my_subscriber
    add_server
    add_client
    action_server
    action_client
    parameter_demo
    executor_demo
    get_device_status_server
    get_device_status_client
    device_status_publisher
    device_status_subscriber

    # 等真正添加 TF2 目标后，再把它加入这里：
    tf2_broadcaster_demo
    tf2_listener_demo
    tf2_point_transform_demo
    tf2_odom_broadcaster_demo
    
    # 旋转四元数
    quaternion_demo
    # 模拟发布障碍信息
    laser_point_publisher
    # 坐标转换
    laser_point_transformer

    #调度器
    executor_single_demo
    executor_multi_demo

    # 声明周期管理
    lifecycle_demo

    # launch节点管理
    status_node

  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# 十一、测试配置
# ============================================================

# 只有在启用 BUILD_TESTING 时才执行以下配置
if(BUILD_TESTING)

  # 查找 ROS 2 的代码规范检查工具
  find_package(ament_lint_auto REQUIRED)

  # 暂时跳过版权声明检查
  set(ament_cmake_copyright_FOUND TRUE)

  # 暂时跳过 cpplint 检查
  set(ament_cmake_cpplint_FOUND TRUE)

  # 加载其他可用的自动检查配置
  ament_lint_auto_find_test_dependencies()
endif()


# ============================================================
# 十二、安装 Launch 文件和配置文件
# ============================================================

# 将 launch 和 config 目录安装到包的 share 目录下
# 这样安装后就能通过 ROS 2 包路径找到这些文件
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


# ============================================================
# 十三、结束 ament 包配置
# ============================================================

# 必须放在 CMakeLists.txt 的最后
ament_package()

