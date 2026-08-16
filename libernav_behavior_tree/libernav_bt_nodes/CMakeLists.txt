cmake_minimum_required(VERSION 3.10)
project(libernav_bt_nodes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(DEPS
  rclcpp
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  geometry_msgs
  behaviortree_cpp_v3
)

include_directories(include)

# ── BT node shared library ────────────────────────────────────────────────────
# Loaded at runtime via BehaviorTreeEngine::loadBTLibraries() / bt_navigator's
# plugin_lib_names parameter — NOT via pluginlib. BT_REGISTER_NODES macro
# inside the .cpp handles self-registration when the .so is dlopen'd.
add_library(libernav_get_path_action SHARED
  src/get_path_action.cpp
)
ament_target_dependencies(libernav_get_path_action ${DEPS})
target_compile_definitions(libernav_get_path_action PRIVATE BT_PLUGIN_EXPORT)

add_library(libernav_exe_path_action SHARED
  src/exe_path_action.cpp
)
ament_target_dependencies(libernav_exe_path_action ${DEPS})
target_compile_definitions(libernav_exe_path_action PRIVATE BT_PLUGIN_EXPORT)

add_library(libernav_recovery_action SHARED
  src/recovery_action.cpp
)
ament_target_dependencies(libernav_recovery_action ${DEPS})
target_compile_definitions(libernav_recovery_action PRIVATE BT_PLUGIN_EXPORT)

add_library(libernav_check_cost_service SHARED
  src/check_cost_service.cpp
)
ament_target_dependencies(libernav_check_cost_service ${DEPS})
target_compile_definitions(libernav_check_cost_service PRIVATE BT_PLUGIN_EXPORT)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS libernav_get_path_action
  libernav_exe_path_action libernav_recovery_action libernav_check_cost_service
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES libernav_tree_nodes.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(libernav_get_path_action
  libernav_exe_path_action libernav_recovery_action libernav_check_cost_service)
ament_export_dependencies(${DEPS})

ament_package()
