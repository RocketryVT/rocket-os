cmake_minimum_required(VERSION 2.8.3)
project(hardware)

## Compile as C++11, supported in ROS Kinetic and newer
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

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorCommand.msg
  SolenoidCommand.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
  LIBRARIES hardware
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# catkin_install_python(PROGRAMS
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Declare a C++ executable

add_executable(motor_driver src/motor_driver.cpp)
add_dependencies(motor_driver hardware_generate_messages_cpp)
target_link_libraries(motor_driver ${catkin_LIBRARIES})

add_executable(solenoid_driver src/solenoid_driver.cpp)
add_dependencies(solenoid_driver hardware_generate_messages_cpp)
target_link_libraries(solenoid_driver ${catkin_LIBRARIES})

add_executable(thermocouple_driver src/thermocouple_driver.cpp)
add_dependencies(thermocouple_driver hardware_generate_messages_cpp)
target_link_libraries(thermocouple_driver ${catkin_LIBRARIES})

add_executable(pressure_transducer_driver src/pressure_transducer_driver.cpp)
add_dependencies(pressure_transducer_driver hardware_generate_messages_cpp)
target_link_libraries(pressure_transducer_driver ${catkin_LIBRARIES})

add_executable(float_switch_driver src/float_switch_driver.cpp)
add_dependencies(float_switch_driver hardware_generate_messages_cpp)
target_link_libraries(float_switch_driver ${catkin_LIBRARIES})

add_executable(imu_driver src/imu_driver.cpp)
add_dependencies(imu_driver hardware_generate_messages_cpp)
target_link_libraries(imu_driver ${catkin_LIBRARIES})
