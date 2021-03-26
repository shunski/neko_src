cmake_minimum_required(VERSION 3.0.2)
project(jcatty_parts_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  jcatty_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jcatty_lib
  CATKIN_DEPENDS roscpp jcatty_msgs
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/BrushedMotor.cpp
  src/${PROJECT_NAME}/BrushlessMotor.cpp
  src/${PROJECT_NAME}/Encoder.cpp
  src/${PROJECT_NAME}/GyroSensor.cpp
  src/${PROJECT_NAME}/KondoServo.cpp
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
