cmake_minimum_required(VERSION 2.8.3)
project(robotic_tutorial)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_service_files(
  FILES
  Fibonacci.srv
)

catkin_python_setup()
generate_messages(
  DEPENDENCIES
    std_msgs
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
