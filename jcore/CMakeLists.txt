cmake_minimum_required(VERSION 3.0.2)
project(jcore)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  jcatty_lib
  jcatty_msgs
  jcatty_action
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jcore
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/jcore.cpp
# )
