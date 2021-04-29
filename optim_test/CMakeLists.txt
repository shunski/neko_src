cmake_minimum_required(VERSION 3.0.2)
project(optim_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  node_lib
)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS roscpp body_lib parts_lib
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable( mc_node src/just_a_mc_node.cpp )
target_link_libraries( mc_node ${catkin_LIBRARIES} )
add_dependencies( mc_node ${catkin_EXPORTED_TARGETS} )

add_executable( fp_node src/just_a_fp_node.cpp )
target_link_libraries( fp_node ${catkin_LIBRARIES} )
add_dependencies( fp_node ${catkin_EXPORTED_TARGETS} )

add_executable( lazy_teensy src/lazy_virtual_teensy.cpp )
target_link_libraries( lazy_teensy ${catkin_LIBRARIES} )
add_dependencies( lazy_teensy ${catkin_EXPORTED_TARGETS} )

add_executable( perfect_teensy src/perfect_virtual_teensy.cpp )
target_link_libraries( perfect_teensy ${catkin_LIBRARIES} )
add_dependencies( perfect_teensy ${catkin_EXPORTED_TARGETS} )

add_executable( action_test_client_node src/action_test_client_node.cpp )
target_link_libraries( action_test_client_node ${catkin_LIBRARIES} )
add_dependencies( action_test_client_node ${catkin_EXPORTED_TARGETS} )

add_executable( core_node src/just_a_core_node.cpp )
target_link_libraries( core_node ${catkin_LIBRARIES} )
add_dependencies( core_node ${catkin_EXPORTED_TARGETS} )
