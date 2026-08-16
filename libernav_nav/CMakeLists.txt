cmake_minimum_required(VERSION 3.10)
project(libernav_nav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(libernav_rog_map REQUIRED)
find_package(libernav_rogmap_core REQUIRED)
find_package(libernav_utility REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

set(LIBERNAV_NAV_SOURCES
  src/costmap_nav/behavior_action_costmap_server.cpp
  src/costmap_nav/controller_action_costmap_server.cpp
  src/costmap_nav/libernav_costmap_nav.cpp
  src/costmap_nav/planner_action_costmap_server.cpp
  src/rogmap_nav/behavior_action_rogmap_server.cpp
  src/rogmap_nav/controller_action_rogmap_server.cpp
  src/rogmap_nav/libernav_rogmap_nav.cpp
  src/rogmap_nav/planner_action_rogmap_server.cpp
  src/libernav_base/behavior_action.cpp
  src/libernav_base/behavior_execution.cpp
  src/libernav_base/controller_action.cpp
  src/libernav_base/controller_execution.cpp
  src/libernav_base/libernav_execution_base.cpp
  src/libernav_base/planner_action.cpp
  src/libernav_base/planner_execution.cpp)

set(LIBERNAV_NAV_DEPS
  angles geometry_msgs nav_2d_msgs nav_2d_utils nav2_core nav2_costmap_2d
  nav2_msgs nav2_util nav_msgs libernav_rog_map libernav_rogmap_core
  libernav_utility pluginlib rclcpp rclcpp_action rclcpp_components
  rclcpp_lifecycle tf2_ros)

add_library(${PROJECT_NAME} SHARED ${LIBERNAV_NAV_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${LIBERNAV_NAV_DEPS})
rclcpp_components_register_nodes(${PROJECT_NAME}
  "libernav_nav::CostmapNavNode"
  "libernav_nav::RogMapNavNode")

add_executable(libernav_nav_node src/main.cpp)
target_compile_features(libernav_nav_node PUBLIC cxx_std_17)
ament_target_dependencies(libernav_nav_node ${LIBERNAV_NAV_DEPS})
target_link_libraries(libernav_nav_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} libernav_nav_node EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(LANGUAGE CPP include src)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${LIBERNAV_NAV_DEPS})
ament_package()
