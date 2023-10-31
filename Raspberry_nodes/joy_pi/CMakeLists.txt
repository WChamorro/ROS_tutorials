cmake_minimum_required(VERSION 3.0.2)
project(joy_pi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
)

find_library(wiringpi_libraries names wiringpi)


catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${WIRINGPI_INCLUDE_DIRS}
)


add_executable(joy_turtle src/joy_turtle.cpp
)
target_link_libraries(joy_turtle ${catkin_LIBRARIES} -lwiringPi
)

