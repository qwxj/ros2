cmake_minimum_required(VERSION 3.8)
project(mini_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

# ============================================================
# 核心库：纯 C++，零 ROS 依赖 → 可被 gtest 秒测
# ============================================================
add_library(mini_slam_core
  src/occupancy_grid.cpp
  src/distance_field.cpp
  src/ekf.cpp
  src/particle_filter.cpp
  src/scan_matcher.cpp
  src/pose_graph.cpp
  src/map_io.cpp
)
target_include_directories(mini_slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mini_slam_core PUBLIC Eigen3::Eigen yaml-cpp)
target_compile_options(mini_slam_core PRIVATE -Wall -Wextra)

# ============================================================
# ROS 节点（把算法接成能跑的东西）
# ============================================================
set(MINI_SLAM_DEPS rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros std_srvs)

add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node mini_slam_core)
ament_target_dependencies(ekf_node ${MINI_SLAM_DEPS})

add_executable(amcl_mini_node src/amcl_mini_node.cpp)
target_link_libraries(amcl_mini_node mini_slam_core)
ament_target_dependencies(amcl_mini_node ${MINI_SLAM_DEPS})

add_executable(slam_mini_node src/slam_mini_node.cpp)
target_link_libraries(slam_mini_node mini_slam_core)
ament_target_dependencies(slam_mini_node ${MINI_SLAM_DEPS})

install(TARGETS ekf_node amcl_mini_node slam_mini_node
  DESTINATION lib/${PROJECT_NAME})


# ============================================================
# 测试
# ============================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_occupancy_grid test/test_occupancy_grid.cpp)
  target_link_libraries(test_occupancy_grid mini_slam_core)

  ament_add_gtest(test_ekf test/test_ekf.cpp)          # ← 新增
  target_link_libraries(test_ekf mini_slam_core)       # ← 新增

  ament_add_gtest(test_pf test/test_pf.cpp)      # ← 新增
  target_link_libraries(test_pf mini_slam_core)   # ← 新增

  ament_add_gtest(test_sm test/test_sm.cpp)      # ← 新增
  target_link_libraries(test_sm mini_slam_core)   # ← 新增

  ament_add_gtest(test_pg test/test_pg.cpp)      # ← 新增
  target_link_libraries(test_pg mini_slam_core)   # ← 新增

  ament_add_gtest(test_map_io test/test_map_io.cpp)      # ← 新增
  target_link_libraries(test_map_io mini_slam_core)

endif()

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mini_slam_core
  EXPORT export_mini_slam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_mini_slam HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()