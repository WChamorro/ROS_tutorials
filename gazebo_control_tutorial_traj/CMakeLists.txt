cmake_minimum_required(VERSION 2.8.3)
project(gazebo_control_tutorial)
 
add_definitions(-std=c++11 -Wall -Wextra )#-Wpedantic -Werror)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  actionlib
  actionlib_msgs
  control_msgs
  std_msgs
  message_generation
  seven_dof_arm_gazebo
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    seven_dof_arm_gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


# Declare the executable, along with its source files.
add_executable(pubcontrols src/pubcontrols.cpp)
add_executable(pubcontrols_traj src/pubcontrols_traj.cpp)
add_executable(pubcontrols_traj_action src/pubcontrols_traj_action.cpp)

# Specify libraries against which to link.
target_link_libraries(pubcontrols ${catkin_LIBRARIES})
target_link_libraries(pubcontrols_traj ${catkin_LIBRARIES})
target_link_libraries(pubcontrols_traj_action ${catkin_LIBRARIES})
