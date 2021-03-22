cmake_minimum_required(VERSION 3.0.2)
project(rh_motion_controller)

find_package(catkin REQUIRED COMPONENTS
  rosdep
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_controller
#  CATKIN_DEPENDS rosdep rospy std_msgs
#  DEPENDS system_lib
)

# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rh_mc_node, RHMotionController.cpp)
target_link_libraries(rh_node, ${catkin_LIBRARIES})
add_dependencies(rh_node, ${catkin_EXPORTED_TARGES})
