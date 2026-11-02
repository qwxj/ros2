
cmake_minimum_required(VERSION 3.8)
project(my_robot_control)

# 使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# 编译动态库
add_library(my_robot_control SHARED
    src/diff_drive_hardware.cpp
    src/imu_sensor.cpp
)

# 头文件目录
target_include_directories(my_robot_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 关联 ROS 2 依赖
ament_target_dependencies(my_robot_control
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

# 注册插件描述文件
pluginlib_export_plugin_description_file(
    hardware_interface
    my_robot_hardware_plugin.xml
)


# 激光雷达
add_executable(laser_scan_simulator
    src/laser_scan_simulator.cpp
)

ament_target_dependencies(laser_scan_simulator
    rclcpp
    sensor_msgs
    tf2
    tf2_ros
    geometry_msgs
    tf2_geometry_msgs
)


install(
    TARGETS laser_scan_simulator
    DESTINATION lib/${PROJECT_NAME}
)

# 安装动态库
install(
    TARGETS my_robot_control
    EXPORT export_my_robot_control
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装头文件
install(
    DIRECTORY include/
    DESTINATION include
)

# 导出库和依赖
ament_export_targets(export_my_robot_control HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

install(
    DIRECTORY urdf config launch maps
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
