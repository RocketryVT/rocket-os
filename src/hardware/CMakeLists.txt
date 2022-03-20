cmake_minimum_required(VERSION 2.8.3)
project(hardware)

find_package(
  catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  geometry_msgs
  genmsg
  roslaunch
)

add_message_files(
  FILES
  DriverCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

