cmake_minimum_required(VERSION 3.0.2)
project(swarm_robot_tb3)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swarmrobot_tb3
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(swarm_robot_control_angle src/swarm_robot_control_angle.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(swarm_robot_control_angle ${catkin_LIBRARIES})


