cmake_minimum_required(VERSION 2.8.3)
project(mastering_robot_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
  geometry_msgs
  message_generation
  tf2 
  tf2_ros
)


#Generate services in the 'srv' folder
add_service_files(
  FILES
  changescale.srv
  changecontrolledjoints.srv
)



## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS tf2 tf2_ros roscpp rospy std_msgs message_runtime

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(robot_description src/joint_publiser.cpp)
target_link_libraries(robot_description ${catkin_LIBRARIES})



