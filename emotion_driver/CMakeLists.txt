cmake_minimum_required(VERSION 2.8.3)
project(emotion_driver)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(emotion_driver src/emotion_driver.cpp)
add_executable(emotion_info src/emotion_info.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(emotion_driver ${catkin_LIBRARIES})
target_link_libraries(emotion_info ${catkin_LIBRARIES})
