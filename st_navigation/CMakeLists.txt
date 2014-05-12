cmake_minimum_required(VERSION 2.8.3)
project(st_navigation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  move_base_msgs 
  actionlib
  tf
  st_topological_mapping
)

catkin_package(
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_base_topo src/move_base_topo.cpp src/shortest_paths.cpp)
target_link_libraries(move_base_topo ${catkin_LIBRARIES})
