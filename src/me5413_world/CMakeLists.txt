cmake_minimum_required(VERSION 3.0.2)
project(me5413_world)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES me5413_world
  # CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs visualization_msgs
  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add Nodes
add_executable(goal_publisher_node src/goal_publisher_node.cpp)
target_link_libraries(goal_publisher_node ${catkin_LIBRARIES})
add_dependencies(goal_publisher_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

