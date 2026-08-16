cmake_minimum_required(VERSION 3.8)
project(navflex_3d_pcd_simulator)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pcd_terrain_simulator src/pcd_terrain_simulator.cpp)
target_compile_features(pcd_terrain_simulator PUBLIC cxx_std_17)
ament_target_dependencies(pcd_terrain_simulator
  geometry_msgs nav_msgs pcl_conversions rclcpp rosgraph_msgs sensor_msgs tf2 tf2_ros)
target_link_libraries(pcd_terrain_simulator ${PCL_LIBRARIES})
target_include_directories(pcd_terrain_simulator SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

install(TARGETS pcd_terrain_simulator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(LANGUAGE CPP src)
endif()

ament_package()
