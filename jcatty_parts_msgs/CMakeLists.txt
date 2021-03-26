cmake_minimum_required(VERSION 3.0.2)
project(jcatty_parts_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)


add_message_files(
  FILES
  KondoServoMsg.msg
  BrushedMotorMsg.msg
  BrushlessMotorMsg.msg
  GyroSensor.msg
  Encoder.msg
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
