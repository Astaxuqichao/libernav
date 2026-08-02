cmake_minimum_required(VERSION 3.8)
project(navflex_frontier_planner)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(navflex_rog_map REQUIRED)
find_package(navflex_rogmap_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

nav2_package()
set(library_name navflex_frontier_planner)
set(dependencies
  geometry_msgs nav_msgs nav2_util navflex_rog_map navflex_rogmap_core
  pluginlib rclcpp rclcpp_lifecycle tf2_ros visualization_msgs)

add_library(${library_name} SHARED
  src/frontier_tools.cpp
  src/candidate_frontier_planner.cpp
  src/frontier_astar_planner.cpp)
target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${library_name} ${dependencies})

pluginlib_export_plugin_description_file(
  navflex_rogmap_core global_planner_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES global_planner_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
