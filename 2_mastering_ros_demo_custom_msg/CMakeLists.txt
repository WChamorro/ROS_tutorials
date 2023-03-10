cmake_minimum_required(VERSION 2.8.3)
project(mastering_ros_demo_custom_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  demo_msg.msg
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#Topic publisher
add_executable(demo_custom_publisher src/demo_custom_msg_publisher.cpp)
add_dependencies(demo_custom_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(demo_custom_publisher ${catkin_LIBRARIES})

#Topic subscriber
add_executable(demo_custom_subscriber src/demo_custom_msg_subscriber.cpp)
add_dependencies(demo_custom_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(demo_custom_subscriber ${catkin_LIBRARIES})

#Topic publisher with params
add_executable(demo_custom_publisher_param src/demo_custom_msg_params.cpp)
add_dependencies(demo_custom_publisher_param ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(demo_custom_publisher_param ${catkin_LIBRARIES})
