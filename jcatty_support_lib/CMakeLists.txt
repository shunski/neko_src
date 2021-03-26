cmake_minimum_required(VERSION 3.0.2)
project(jcatty_support_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  jcatty_msgs
  jcatty_action
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jcatty_lib
  CATKIN_DEPENDS roscpp std_msgs
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/Calibration.cpp
  src/${PROJECT_NAME}/Plot.cpp
  src/${PROJECT_NAME}/Secretary.cpp
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
