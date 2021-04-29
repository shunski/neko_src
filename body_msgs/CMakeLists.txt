cmake_minimum_required(VERSION 3.0.2)
project(body_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  parts_msgs
  message_generation
)


add_message_files(
  FILES
  PartMsg.msg
  PartCommandMsg.msg
  ProcessedFeedbackMsg.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  parts_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jcatty_simple_struct_msgs
 CATKIN_DEPENDS std_msgs parts_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
