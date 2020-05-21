cmake_minimum_required(VERSION 2.8.3)
project(trajectory_pub)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  control_msgs
  mav_trajectory_generation
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs tf mav_planning_msgs control_msgs mavros_msgs
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

############
# BINARIES #
############
add_executable(trajectory_pub
        src/trajectoryPublisher.cpp
        src/trajectoryPublisher_node.cpp
        src/eth_trajectory.cpp
        src/flip_trajectory.cpp
)

add_executable(flip_trajectory_pub
        src/trajectoryPublisher.cpp
        src/eth_trajectory.cpp
        src/flip_node.cpp
        src/flip_trajectory.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


add_dependencies(flip_trajectory_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(flip_trajectory_pub ${catkin_LIBRARIES})