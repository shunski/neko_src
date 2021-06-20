cmake_minimum_required(VERSION 3.0.2)
project(actuator_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  support_lib
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
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES actuator_lib
  CATKIN_DEPENDS roscpp support_lib
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Motor.cpp
  src/KondoServo.cpp
)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
