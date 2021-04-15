cmake_minimum_required(VERSION 3.0.2)
project(parts_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  parts_msgs
  support_lib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES parts_lib
  CATKIN_DEPENDS roscpp parts_msgs support_lib
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/BrushedMotor.cpp
  src/BrushlessMotor.cpp
  src/Encoder.cpp
  src/MotionSensor.cpp
  src/KondoServo.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}$
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}$
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
