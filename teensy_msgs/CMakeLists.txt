cmake_minimum_required(VERSION 3.0.2)
project(teensy_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  actuator_lib
  message_generation
)


add_message_files(
  FILES
  CommandMsg.msg
  FeedbackMsg.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  actuator_lib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS std_msgs actuator_lib message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
