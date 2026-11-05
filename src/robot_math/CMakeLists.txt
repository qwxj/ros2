cmake_minimum_required(VERSION 3.8)
project(robot_math)

# L0 练习包：只做三件事 —— 编库、装头文件、导出 target。
# 注意：这个包没有 find_package(rclcpp)，它就是一个普通 C++ 库。

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

add_library(robot_math SHARED
  src/diff_drive.cpp
)

# BUILD_INTERFACE：在源码树里编译时用源码的 include/
# INSTALL_INTERFACE：被安装后，消费端用 install/include
target_include_directories(robot_math PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_compile_options(robot_math PRIVATE -Wall -Wextra -Wpedantic)

# 1) 装库，并把库登记进名为 export_robot_math 的导出集
install(
  TARGETS robot_math
  EXPORT export_robot_math
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 2) 装头文件（目录结构与源码树一致：include/robot_math/*.hpp）
install(
  DIRECTORY include/
  DESTINATION include
)

# 3) 导出：生成 share/robot_math/cmake/robot_mathConfig.cmake，
#    让别的包 find_package(robot_math) 之后能拿到 robot_math::robot_math 这个 imported target
ament_export_targets(export_robot_math HAS_LIBRARY_TARGET)

# 4) 声明"消费我的人也需要 ament_cmake"（本包无其它依赖，故只有这一条）
ament_export_dependencies(ament_cmake)

ament_package()
