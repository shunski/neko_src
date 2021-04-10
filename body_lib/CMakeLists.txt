cmake_minimum_required(VERSION 3.0.2)
project(body_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  support_lib
  parts_msgs
  parts_lib
  body_msgs
  motioncontroll_action
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES body_lib
  CATKIN_DEPENDS roscpp
  DEPENDS parts_lib parts_msgs body_msgs motioncontroll_action
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Part.cpp
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
