cmake_minimum_required(VERSION 2.8.3)
project(cartesian_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  rospy
  sensor_msgs
  tf
  urdfdom_py
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cartesian_control
  CATKIN_DEPENDS 
    interactive_markers 
    rospy
    sensor_msgs
    tf
    urdfdom_py
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(cartesian_control
#   src/${PROJECT_NAME}/cartesian_control.cpp
# )

## Declare a cpp executable
# add_executable(cartesian_control_node src/cartesian_control_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(cartesian_control_node cartesian_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(cartesian_control_node
#   ${catkin_LIBRARIES}
# )
