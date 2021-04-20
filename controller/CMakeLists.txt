cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  node_lib
)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS roscpp body_lib parts_lib
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable( just_a_motion_controller src/McNode.cpp )
target_link_libraries( just_a_motion_controller ${catkin_LIBRARIES} )
add_dependencies( just_a_motion_controller ${catkin_EXPORTED_TARGETS} )

add_executable( just_a_feedback_processor src/FpNode.cpp )
target_link_libraries( just_a_feedback_processor ${catkin_LIBRARIES} )
add_dependencies( just_a_feedback_processor ${catkin_EXPORTED_TARGETS} )
