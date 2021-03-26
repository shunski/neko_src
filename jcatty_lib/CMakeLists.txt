cmake_minimum_required(VERSION 3.0.2)
project(jcatty_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs

  jcatty_msgs
  jcatty_action

  jcatty_parts_lib
  jcatty_body_lib
  jcatty_locomotion_lib
  jcatty_support_lib
)
catkin_metapackage()
