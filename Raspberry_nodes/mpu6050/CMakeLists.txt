cmake_minimum_required(VERSION 3.0.2)
project(mpu6050)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)
find_package(Eigen3 REQUIRED)

find_library(wiringpi_libraries names wiringpi)


catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs
)



include_directories(
  include/mpu6050
  ${catkin_INCLUDE_DIRS}
  ${WIRINGPI_INCLUDE_DIRS}
)


add_executable(imu_node src/read_MPU6050Pi.cpp
src/I2CPi.cpp
src/math_3d.cpp
src/MPU6050Pi.cpp


)
target_link_libraries(imu_node ${catkin_LIBRARIES} -lwiringPi
)

