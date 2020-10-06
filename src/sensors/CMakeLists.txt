cmake_minimum_required(VERSION 2.8.3)
project(sensors)

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
  SensorReading.msg
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

