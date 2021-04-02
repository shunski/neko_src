cmake_minimum_required(VERSION 3.0.2)
project(jcatty_teensy_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  jcatty_parts_msgs
  message_generation
)


add_message_files(
  FILES
  ForelegCommandMsg.msg
  ForeleginfoMsg.msg
  HindlegCommandMsg.msg
  HindleginfoMsg.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  message_generation
  jcatty_parts_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
 CATKIN_DEPENDS std_msgs jcatty_parts_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
