cmake_minimum_required(VERSION 2.8.3)
project(models)

add_compile_options(-std=c++11)

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
  include/math
  include/modeling
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(models
  src/math/axis_angle.cpp
  src/math/complex.cpp
  src/math/echelon.cpp
  src/math/matrix.cpp
  src/math/dynamic_matrix.cpp
  src/math/quaternion.cpp
  src/math/solve.cpp
  src/modeling/standard_atmosphere.cpp
  src/modeling/compute_drag.cpp
  src/modeling/hybrid_thrust.cpp
  src/modeling/cubic_spline.cpp
  src/modeling/secant_root_solve.cpp
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
  tests/catch_main.cpp
  tests/complex_tests.cpp
  tests/kalman_tests.cpp
  tests/matrix_tests.cpp
  tests/dynamic_matrix_tests.cpp
  tests/solve_tests.cpp
  tests/runge_kutta_tests.cpp
)

target_link_librarieS(unit_tests models)
