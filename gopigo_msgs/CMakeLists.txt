cmake_minimum_required(VERSION 2.8.3)
project(gopigo_msgs)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
   FILES  GopigoState.msg 
)
generate_messages(
   DEPENDENCIES  std_msgs geometry_msgs sensor_msgs
)
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
)
