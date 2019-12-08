cmake_minimum_required(VERSION 2.8.3)
project(robot_sim_prj5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  trajectory_msgs
  urdf
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_sim_prj5
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    trajectory_msgs
    urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(robot_sim_prj5
  src/joint_state_publisher.cpp
  src/robot.cpp
  src/velocity_controller.cpp
  src/position_controller.cpp
  src/trajectory_executer.cpp
)

add_executable(robot_sim_bringup_prj5 src/robot_sim_bringup.cpp)
target_link_libraries(robot_sim_bringup_prj5 ${catkin_LIBRARIES} robot_sim_prj5)
