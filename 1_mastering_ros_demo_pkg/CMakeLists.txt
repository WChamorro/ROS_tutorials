cmake_minimum_required(VERSION 2.8.3)
project(mastering_ros_demo_std_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#Topic publisher
add_executable(demo_publisher src/demo_std_msg_publisher.cpp)
add_dependencies(demo_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(demo_publisher ${catkin_LIBRARIES})

#Topic publisher
add_executable(demo_subscriber src/demo_std_msg_subscriber.cpp)
add_dependencies(demo_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(demo_subscriber ${catkin_LIBRARIES})
