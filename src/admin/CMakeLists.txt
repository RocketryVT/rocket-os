cmake_minimum_required(VERSION 2.8.3)
project(admin)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  genmsg
  roslaunch
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

