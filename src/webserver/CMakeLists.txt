cmake_minimum_required(VERSION 2.8.3)
project(webserver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
)

catkin_package()

catkin_install_python(PROGRAMS scripts/webserver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
