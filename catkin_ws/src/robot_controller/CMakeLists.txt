cmake_minimum_required(VERSION 2.8.3)
project(robot_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
  nav_msgs
)

## Find Gazebo
find_package(gazebo REQUIRED)

## Find PCL
find_package(PCL REQUIRED)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs tf pcl_ros pcl_conversions nav_msgs
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

## Add definitions
add_definitions(
  ${PCL_DEFINITIONS}
  ${GAZEBO_CXX_FLAGS}
)

## Source files
set(SOURCES
  src/teleop_robot.cpp
  src/random_patrol_and_obstacle_avoidance.cpp
  src/patrol_with_obstacle_avoidance.cpp
)

## Executable for teleoperation
add_executable(teleop_robot_node src/teleop_robot.cpp)
target_link_libraries(teleop_robot_node
  ${catkin_LIBRARIES}
)

## Executable for patrol with obstacle avoidance
add_executable(patrol_with_obstacle_avoidance_node src/patrol_with_obstacle_avoidance.cpp)
target_link_libraries(patrol_with_obstacle_avoidance_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Executable for random patrol and obstacle avoidance
add_executable(random_patrol_and_obstacle_avoidance_node src/random_patrol_and_obstacle_avoidance.cpp)
target_link_libraries(random_patrol_and_obstacle_avoidance_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Install executables and headers
install(TARGETS random_patrol_and_obstacle_avoidance_node teleop_robot_node patrol_with_obstacle_avoidance_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

