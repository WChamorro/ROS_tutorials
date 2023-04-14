cmake_minimum_required(VERSION 2.8.3)
project(mastering_ros_demo_services)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  demo_srv.srv
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
)


#Demo service server
add_executable(demo_service_server src/demo_service_server.cpp)
add_dependencies(demo_service_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(demo_service_server ${catkin_LIBRARIES})

#Demo service client
add_executable(demo_service_client src/demo_service_client.cpp)
add_dependencies(demo_service_client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(demo_service_client ${catkin_LIBRARIES})

