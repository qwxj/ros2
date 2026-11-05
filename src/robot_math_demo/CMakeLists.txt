cmake_minimum_required(VERSION 3.8)
project(robot_math_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 关键行：这一行能成功，说明 robot_math 的导出是对的。
# 如果报 "Could not find a package configuration file provided by robot_math"，
# 说明 robot_math 没编译，或者编译后没 source install/setup.bash。
find_package(robot_math REQUIRED)

add_executable(wheel_speed_node src/demo_node.cpp)

# 两种写法混用，正好对照：
#   ament_target_dependencies —— 老式，读 <pkg>_INCLUDE_DIRS / <pkg>_LIBRARIES / <pkg>_TARGETS
#   target_link_libraries(robot_math::robot_math) —— 新式，imported target 自带 interface
ament_target_dependencies(wheel_speed_node
  rclcpp
  geometry_msgs
  std_msgs
)
target_link_libraries(wheel_speed_node robot_math::robot_math)

# 一定要装到 lib/${PROJECT_NAME}，ros2 run 才找得到。
# 改成 DESTINATION bin 做一次破坏性实验（见 docs/learning/L0-ament-colcon.md 实验 C-5）。
install(
  TARGETS wheel_speed_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
