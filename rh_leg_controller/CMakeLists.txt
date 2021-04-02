cmake_minimum_required(VERSION 3.0.2)
project(rh_leg_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  jcatty_lib
  jcatty_msgs
  jcatty_action
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

add_executable(rh_motion_controller, main.cpp)
target_link_libraries(rh_motion_controller, ${catkin_LIBRARIES})
add_dependencies(rh_motion_controller, ${catkin_EXPORTED_TARGES})
