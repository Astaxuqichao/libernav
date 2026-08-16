cmake_minimum_required(VERSION 3.10)
project(libernav_rogmap_bt_navigator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(rogmap_bt_navigator src/rogmap_bt_navigator.cpp)
ament_target_dependencies(rogmap_bt_navigator
  ament_index_cpp
  behaviortree_cpp_v3
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros)

install(TARGETS rogmap_bt_navigator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees params
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
