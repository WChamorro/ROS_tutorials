cmake_minimum_required(VERSION 2.8.3)
project(gazebo_sensors_tutorial)
 
add_definitions(-std=c++11 -Wall -Wextra -Wdeprecated)#-Wpedantic -Werror)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  aruco_ros
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    aruco_ros
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(aruco_frames_detection src/aruco_frames_detection.cpp)
target_link_libraries(aruco_frames_detection ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

