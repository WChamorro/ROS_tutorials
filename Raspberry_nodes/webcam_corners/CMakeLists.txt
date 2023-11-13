cmake_minimum_required(VERSION 3.0.2)
project(webcam_corners)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  image_transport
  camera_info_manager
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(corners src/corners.cpp)
target_link_libraries(corners ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


