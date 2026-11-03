cmake_minimum_required(VERSION 3.8)
project(interview_practice)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(DEPS
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

# 每个可执行文件一个源文件；注释掉的等你粘贴后再打开
set(NODES
  goto_point
  # nearest_obstacle
  # pose_reporter
  # obstacle_server
  # cmd_vel_watchdog
)

foreach(node ${NODES})
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${node}.cpp)
    add_executable(${node} src/${node}.cpp)
    ament_target_dependencies(${node} ${DEPS})
    install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
    message(STATUS "[interview_practice] 找到并编译: ${node}.cpp")
  else()
    message(STATUS "[interview_practice] 跳过（文件不存在）: ${node}.cpp")
  endif()
endforeach()

ament_package()