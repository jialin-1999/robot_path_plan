cmake_minimum_required(VERSION 2.8.3)
project(csc_nav2d_navigator)
SET(CMAKE_CXX_FLAGS "-std=c++11 -g -O3 ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  tf
  geometry_msgs
  costmap_2d
  serial
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
#add_action_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  actionlib_msgs
#  geometry_msgs
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES RobotNavigator MapInflationTool
  CATKIN_DEPENDS
    roscpp
    message_runtime
    tf
    sensor_msgs
    geometry_msgs
    costmap_2d
    serial
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(MapInflationTool src/MapInflationTool.cpp)

## Declare a cpp executable

add_executable(local_navigator src/localRobotNavigator.cpp)
#add_executable(local_navigator src/localRobotNavigator_cxy.cpp)
add_executable(set_target src/set_target.cpp)
#add_executable(set_target src/set_target_cxy.cpp)
add_executable(GPStoTF src/GPStoTF.cpp)
add_executable(marker src/marker.cpp)
add_executable(test_GPS_input src/test_GPS_input.cpp)
add_executable(pub_gps_point src/pub_gps_point)
add_executable(rtk_gps src/rtk_gps.cpp)
add_executable(recieve_position src/recieve_position.cpp)
add_executable(waypoint src/waypoint.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(local_navigator ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(set_target ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(GPStoTF ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(test_GPS_input ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(local_navigator MapInflationTool ${catkin_LIBRARIES})
target_link_libraries(set_target MapInflationTool ${catkin_LIBRARIES})
target_link_libraries(GPStoTF MapInflationTool ${catkin_LIBRARIES})
target_link_libraries(marker ${catkin_LIBRARIES})
target_link_libraries(test_GPS_input ${catkin_LIBRARIES})
target_link_libraries(pub_gps_point ${catkin_LIBRARIES})
target_link_libraries(rtk_gps ${catkin_LIBRARIES})
target_link_libraries(recieve_position ${catkin_LIBRARIES})
target_link_libraries(waypoint ${catkin_LIBRARIES})
