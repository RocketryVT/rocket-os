cmake_minimum_required(VERSION 2.8.3)
project(examples)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bare_bones_node src/bare_bones_node.cpp)
target_link_libraries(bare_bones_node ${catkin_LIBRARIES})

