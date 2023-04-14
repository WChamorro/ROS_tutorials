cmake_minimum_required(VERSION 2.8.3)
project(mastering_ros_demo_tf2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  tf2
  geometry_msgs
)





catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#Demo service server
add_executable(turtle_tf_broadcaster src/turtle_tf2_broadcaster.cpp)
add_dependencies(turtle_tf_broadcaster ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})

#Demo service client
add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
add_dependencies(turtle_tf2_listener ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(turtle_tf2_listener ${catkin_LIBRARIES})

