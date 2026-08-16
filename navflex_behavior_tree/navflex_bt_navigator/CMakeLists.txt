cmake_minimum_required(VERSION 3.10)
project(navflex_bt_navigator)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
