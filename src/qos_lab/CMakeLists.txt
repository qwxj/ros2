cmake_minimum_required(VERSION 3.8)
project(qos_lab)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 内部复用：QoS 参数解析 + 打印。静态库，不导出（只有本包用）。
add_library(qos_utils STATIC src/qos_utils.cpp)
target_include_directories(qos_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
ament_target_dependencies(qos_utils rclcpp)

foreach(exe qos_matrix lifespan_lab)
  add_executable(${exe} src/${exe}.cpp)
  target_link_libraries(${exe} qos_utils)
  ament_target_dependencies(${exe} rclcpp std_msgs)
  install(TARGETS ${exe} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
