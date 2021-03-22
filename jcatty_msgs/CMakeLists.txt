cmake_minimum_required(VERSION 3.0.2)
project(jcatty_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  jcatty_simple_struct_msgs
  message_generation
)

add_message_files(
  FILES
   heartrate.msg
   rh_info.msg
   rh_cmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs jcatty_simple_struct_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jcatty_msgs
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
