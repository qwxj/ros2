cmake_minimum_required(VERSION 3.8)
project(my_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ---------- 插件动态库 ----------
add_library(my_hardware SHARED
  src/sine_joint.cpp
)
target_include_directories(my_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(my_hardware
  rclcpp rclcpp_lifecycle hardware_interface pluginlib
)

# ★ 关键：把 plugin xml 注册到 hardware_interface 的插件索引里
pluginlib_export_plugin_description_file(hardware_interface my_hardware_plugin.xml)

# ---------- 安装 ----------
install(TARGETS my_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES my_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(my_hardware)
ament_export_dependencies(ament_cmake rclcpp rclcpp_lifecycle hardware_interface pluginlib)

ament_package()