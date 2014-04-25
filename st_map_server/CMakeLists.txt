cmake_minimum_required(VERSION 2.8.3)
project(st_map_server)

add_definitions(-DDEBUG=1 ) #This allows usage of #IF DEBUG in all code

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  st_topological_mapping
)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(load_map src/load_map.cpp)
add_dependencies(load_map ${catkin_EXPORTED_TARGETS}) #to make sure depended message headers are generated first!
target_link_libraries(load_map ${catkin_LIBRARIES})

add_executable(save_map src/save_map.cpp)
add_dependencies(save_map ${catkin_EXPORTED_TARGETS}) #to make sure  message headers are generated first!
target_link_libraries(save_map ${catkin_LIBRARIES})
