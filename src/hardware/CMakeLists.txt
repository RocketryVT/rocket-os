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

add_executable(motor_driver src/motor_driver.cpp)
target_link_libraries(motor_driver ${catkin_LIBRARIES})

add_executable(solenoid_driver src/solenoid_driver.cpp)
target_link_libraries(solenoid_driver ${catkin_LIBRARIES})

add_executable(thermocouple_driver src/thermocouple_driver.cpp)
target_link_libraries(thermocouple_driver ${catkin_LIBRARIES})

add_executable(pressure_transducer_driver src/pressure_transducer_driver.cpp)
target_link_libraries(pressure_transducer_driver ${catkin_LIBRARIES})

add_executable(float_switch_driver src/float_switch_driver.cpp)
target_link_libraries(float_switch_driver ${catkin_LIBRARIES})

add_executable(imu_driver src/imu_driver.cpp)
target_link_libraries(imu_driver ${catkin_LIBRARIES})

add_executable(barometer_driver src/barometer_driver.cpp)
target_link_libraries(barometer_driver ${catkin_LIBRARIES})
