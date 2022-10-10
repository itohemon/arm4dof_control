cmake_minimum_required(VERSION 3.0.2)
project(arm4dof_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_sdk
  std_msgs
  controller_manager
  hardware_interface
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs controller_manager hardware_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm4dof_control
  src/main.cpp
  src/sts3215_config.cpp
  src/sts3215_hardware.cpp
  src/sts3215_joint.cpp
  src/sts3215_servo.cpp
  src/sts3215_port.cpp
  )

target_link_libraries(arm4dof_control
  ${catkin_LIBRARIES}
  )
