cmake_minimum_required(VERSION 3.0.2)
project(core_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  support_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES support_lib
  CATKIN_DEPENDS roscpp
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Calibrator.cpp
  src/Decoder.cpp
  src/Recoder.cpp
  src/Plot.cpp
  src/Utilities.cpp
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
