cmake_minimum_required(VERSION 2.8.3)
project(st_navigation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  message_generation
  #std_msgs
  #move_base_msgs 
  tf
  st_topological_mapping
)

add_action_files(
  DIRECTORY
    action
  FILES
    GotoNode.action
)

generate_messages(
  DEPENDENCIES 
    std_msgs 
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_base_topo src/move_base_topo.cpp src/shortest_paths.cpp)
target_link_libraries(move_base_topo ${catkin_LIBRARIES})
add_dependencies(move_base_topo st_topological_mapping_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
