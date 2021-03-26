cmake_minimum_required(VERSION 3.0.2)

project(jcatty_action)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  std_msgs
  jcatty_lib

  jcatty_motionControll_action
)

catkin_metapackage()
