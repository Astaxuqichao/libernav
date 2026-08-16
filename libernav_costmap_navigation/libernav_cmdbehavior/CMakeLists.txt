cmake_minimum_required(VERSION 3.10)
project(libernav_cmdbehavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(libernav_cmdbehavior SHARED
  src/cmd_behavior.cpp
)

set(LIBERNAV_BEHAVIORS_DEPS
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  pluginlib
  tf2_ros
)

ament_target_dependencies(libernav_cmdbehavior ${LIBERNAV_BEHAVIORS_DEPS})

pluginlib_export_plugin_description_file(nav2_core libernav_cmdbehavior_plugin.xml)

install(TARGETS libernav_cmdbehavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES libernav_cmdbehavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(libernav_cmdbehavior)
ament_export_dependencies(${LIBERNAV_BEHAVIORS_DEPS})

ament_package()
