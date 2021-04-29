cmake_minimum_required(VERSION 3.0.2)
project(node_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  support_lib
  support_msgs
  body_msgs
  body_lib
  parts_lib
  parts_msgs
  motioncontroll_action
  teensy_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES node_lib
  CATKIN_DEPENDS
    roscpp
    body_lib
    body_msgs
    support_lib
    support_msgs
    std_msgs
    parts_lib
    parts_msgs
    motioncontroll_action
    teensy_msgs
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/HeartrateSubscriberNode.cpp
  src/MotionControllerNode.cpp
  src/FeedbackProcessorNode.cpp
  src/RegistrarNode.cpp
  src/HeartratePublisherNode.cpp
  src/GenericCattyNode.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
