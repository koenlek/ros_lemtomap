cmake_minimum_required(VERSION 2.8.3)
project(lemto_map_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  lemto_topological_mapping
  tf
)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(save_map src/save_map.cpp)
add_dependencies(save_map ${catkin_EXPORTED_TARGETS}) #to make sure  message headers are generated first!
target_link_libraries(save_map ${catkin_LIBRARIES} yaml-cpp)
