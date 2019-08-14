cmake_minimum_required(VERSION 2.8.3)
project(models)

add_compile_options(-std=c++11 -Wfatal-errors)

find_package(catkin REQUIRED
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  genmsg
  roslaunch
)

## Your package locations should be listed before other locations
include_directories(
  include
  lambda/include
  lambda/test
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(models
  lambda/src/axis_angle.cpp
  lambda/src/complex.cpp
  lambda/src/echelon.cpp
  lambda/src/matrix.cpp
  lambda/src/dynamic_matrix.cpp
  lambda/src/quaternion.cpp
  lambda/src/solve.cpp
  src/standard_atmosphere.cpp
  src/compute_drag.cpp
  src/hybrid_thrust.cpp
  src/cubic_spline.cpp
  src/secant_root_solve.cpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES models
)

## Declare a C++ executable
add_executable(pose_estimator src/pose_estimator.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pose_estimator models)
target_link_libraries(pose_estimator ${catkin_LIBRARIES})

add_executable(unit_tests
  lambda/test/catch_main.cpp
  lambda/test/complex_tests.cpp
  lambda/test/kalman_tests.cpp
  lambda/test/matrix_tests.cpp
  lambda/test/dynamic_matrix_tests.cpp
  lambda/test/solve_tests.cpp
  tests/runge_kutta_tests.cpp
)

target_link_libraries(unit_tests models)
