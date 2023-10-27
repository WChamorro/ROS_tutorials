cmake_minimum_required(VERSION 3.0.2)
project(gpio_encoder)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

find_library(wiringpi_libraries names wiringpi)


catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${WIRINGPI_INCLUDE_DIRS}
)


add_executable(encoder src/encoder.cpp

)
target_link_libraries(encoder ${catkin_LIBRARIES} -lwiringPi
)

