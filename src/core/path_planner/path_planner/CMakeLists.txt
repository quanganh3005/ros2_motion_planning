cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)
find_package(common REQUIRED)
find_package(system_config REQUIRED)
find_package(glog REQUIRED)

file(GLOB path_planner_src_files src/*.cpp)
file(GLOB path_planner_utils_src_files src/utils/*.cpp)
file(GLOB graph_path_planner_src_files src/graph_planner/*.cpp)
file(GLOB sample_path_planner_src_files src/sample_planner/*.cpp)
file(GLOB evolutionary_path_planner_src_files src/evolutionary_planner/*.cpp)
file(GLOB graph_path_planner_hybrid_astar_src_files src/graph_planner/hybrid_astar_planner/*.cpp)

add_library(${PROJECT_NAME} SHARED
  ${path_planner_src_files}
  ${path_planner_utils_src_files}
  ${graph_path_planner_hybrid_astar_src_files}
  ${graph_path_planner_src_files}
  ${sample_path_planner_src_files}
  ${evolutionary_path_planner_src_files}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  visualization_msgs
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
  angles
  common
  system_config
)

target_link_libraries(${PROJECT_NAME}
  glog::glog
)

pluginlib_export_plugin_description_file(nav2_core path_planner_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  visualization_msgs
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
  angles
  common
  system_config
)

ament_package()
