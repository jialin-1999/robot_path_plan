cmake_minimum_required(VERSION 2.8.3)
project(speed_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES color_blocks
#  CATKIN_DEPENDS ros_cpp ros_py std_msgs
#  DEPENDS system_lib
)

add_executable(${PROJECT_NAME} src/speed_control.cpp)

target_link_libraries(${PROJECT_NAME}
	 ${catkin_LIBRARIES}
)
