cmake_minimum_required(VERSION 2.8.3)
project(rviz_marker_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  std_msgs
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_executable(rviz_node src/rviz_node.cpp)
target_link_libraries( rviz_node ${catkin_LIBRARIES})
add_dependencies(rviz_node ${catkin_EXPORTED_TARGETS})


