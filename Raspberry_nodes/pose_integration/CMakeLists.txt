cmake_minimum_required(VERSION 3.0.2)
project(pose_integration)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  tf2
  geometry_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)



catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)


#Demo service server
add_executable(pose src/pose.cpp)
target_link_libraries(pose ${catkin_LIBRARIES})

