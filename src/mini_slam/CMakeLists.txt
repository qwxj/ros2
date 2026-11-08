cmake_minimum_required(VERSION 3.8)
project(mini_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ============================================================
# 核心库：纯 C++，零 ROS 依赖 → 可被 gtest 秒测
# ============================================================
add_library(mini_slam_core
  src/occupancy_grid.cpp
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