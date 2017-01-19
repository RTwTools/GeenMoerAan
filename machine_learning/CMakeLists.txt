cmake_minimum_required(VERSION 2.8.3)
project(machine_learning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  std_msgs
  message_generation
  OpenCV
  cv_bridge
  image_transport
)

# Declare ROS messages, services and actions #

## Generate messages in the 'msg' folder
#add_message_files(
 #  FILES
   #Detection.msg
 #)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} )
find_package( OpenCV REQUIRED)

add_executable(machine_learning_node src/machine_learning.cpp)
target_link_libraries(machine_learning_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(machine_learning_node machine_learning_gencpp)
