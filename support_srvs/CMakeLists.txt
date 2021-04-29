cmake_minimum_required(VERSION 3.0.2)
project(support_srvs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)


add_service_files(
  FILES
  RegistrationSrv.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
 CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
