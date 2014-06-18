cmake_minimum_required(VERSION 2.8.3)
project(st_topological_mapping)

# Load the used catkin_make build type here, make available in c++ code for preprocessor.
# This allows usage of #IF DEBUG in all code of this package. It retrieves the value from cmakes CMAKE_BUILD_TYPE. 
# This can be switched using catkin_make -DCMAKE_BUILD_TYPE=Release , or catkin_make -DCMAKE_BUILD_TYPE=Debug
add_definitions(-DCMAKE_BUILD_TYPE_DEF=${CMAKE_BUILD_TYPE}) 
if(CMAKE_BUILD_TYPE MATCHES Debug)
add_definitions(-DDEBUG=1) 
else()
add_definitions(-DDEBUG=0)
endif()

add_definitions(-DBENCHMARKING=1) 

find_package(
catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  tf
  rosbag #for loading previous maps
  ecl_time
)
find_package(Eigen REQUIRED)

add_message_files(
  FILES
  TopologicalNavigationMap.msg
  TopoNavEdgeMsg.msg
  TopoNavNodeMsg.msg
)

generate_messages(   
  DEPENDENCIES  
  std_msgs 
  geometry_msgs
)  

catkin_package(
  CATKIN_DEPENDS message_runtime
  INCLUDE_DIRS include
)

include_directories(
  include 
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(topological_navigation_mapper src/toponav_edge.cpp src/toponav_node.cpp src/utils.cpp src/show_toponav_map.cpp src/toponav_map.cpp src/main.cpp src/load_map.cpp src/bgl/bgl_functions.cpp)
target_link_libraries(topological_navigation_mapper ${catkin_LIBRARIES})
add_dependencies(topological_navigation_mapper ${catkin_EXPORTED_TARGETS}) # KL: to make sure that message headers in devel/include/${PROJECT_NAME} are recognized

