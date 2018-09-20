cmake_minimum_required(VERSION 2.8.3)
project(robotic_tutorial)


find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
