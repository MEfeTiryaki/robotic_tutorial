cmake_minimum_required(VERSION 2.8.3)
project(gopigo_sim)

find_package(catkin REQUIRED
  COMPONENTS
    gopigo_description
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    gopigo_description
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
