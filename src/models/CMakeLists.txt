cmake_minimum_required(VERSION 2.8.3)
project(models)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  message_generation
  roscpp
  rospy
  std_msgs
  genmsg
  roslaunch
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
# )

## Generate services in the 'srv' folder
# add_service_files(
# )

## Generate actions in the 'action' folder
# add_action_files(
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(models
  src/models.cpp
  src/axis_angle.cpp
  src/complex.cpp
  src/echelon.cpp
  src/levenshtein.cpp
  src/matrix.cpp
  src/quaternion.cpp
  src/solve.cpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES models
)

## Declare a C++ executable
add_executable(executable src/executable.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(executable models)
target_link_libraries(executable ${catkin_LIBRARIES})

