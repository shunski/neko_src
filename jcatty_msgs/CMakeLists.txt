cmake_minimum_required(VERSION 3.0.2)
project(jcatty_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  jcatty_bodypart_msgs
  jcatty_parts_msgs
  jcatty_calibration_msgs
)
catkin_metapackage()
