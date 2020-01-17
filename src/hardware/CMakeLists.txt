cmake_minimum_required(VERSION 2.8.3)
project(hardware)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  genmsg
  roslaunch
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

