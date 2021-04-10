cmake_minimum_required(VERSION 3.0.2)
project(parts_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)


add_message_files(
  FILES
  KondoServoCommandMsg.msg
  KondoServoFeedbackMsg.msg
  KondoServoMsg.msg
  BrushedMotorCommandMsg.msg
  BrushedMotorFeedbackMsg.msg
  BrushedMotorMsg.msg
  BrushlessMotorCommandMsg.msg
  BrushlessMotorFeedbackMsg.msg
  BrushlessMotorMsg.msg
  GyroSensorMsg.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jcatty_simple_struct_msgs
 CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
