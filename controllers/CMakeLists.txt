cmake_minimum_required(VERSION 2.8.12)
project(controllers)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES controllers
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs tf
)

#############
# LIBRARIES #
#############
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/controller.cpp
  src/eth_quat_controller.cpp
  src/upenn_controller.cpp
  src/math_operations.cpp 
  src/controller_class.cpp
  src/euler_controller.cpp
  #src/smc_controller.cpp
)
add_dependencies(controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(controller_node
  src/controller_node.cpp
  src/eth_quat_controller.cpp
  src/upenn_controller.cpp
  src/math_operations.cpp 
  src/controller_class.cpp
  src/euler_controller.cpp
  #src/smc_controller.cpp
  #src/controller.cpp
)
add_dependencies(controller_node controller euler_controller eth_quat_controller upenn_controller math_operations controller_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})

