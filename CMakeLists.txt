cmake_minimum_required(VERSION 3.0.2)
project(qt_sermo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


## Uncomment this if the package has a setup.py.
catkin_python_setup() 

## Generate services in the 'srv' folder
add_service_files(
  FILES
  sermo_info.srv
  sermo_chat.srv
  sermo_record.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
