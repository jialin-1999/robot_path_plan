cmake_minimum_required(VERSION 2.8.3)
project(pioneer_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
  laser_geometry)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS})

add_executable (laserscan_to_pointcloud2 src/laserscan_to_pointcloud2.cpp)
target_link_libraries (laserscan_to_pointcloud2 ${catkin_LIBRARIES})
