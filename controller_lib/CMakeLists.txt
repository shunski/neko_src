cmake_minimum_required(VERSION 3.0.2)
project(controller_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  support_lib
  actuator_lib
  teensy_msgs
  message_generation
  actionlib_msgs
)

add_message_files(
  FILES
  BodyMsg.msg
  BodyCommandMsg.msg
  ProcessedFeedbackMsg.msg
)

add_action_files(
  FILES
  MotionControll.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actuator_lib
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES body_lib
  CATKIN_DEPENDS
    roscpp
    support_lib
    actuator_lib
    teensy_msgs
  DEPENDS
    actuator_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Body.cpp
  src/MotionController.cpp
  src/FeedbackProcessor.cpp
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
