cmake_minimum_required(VERSION 2.8.3)
project(st_map_server)

# Load the used catkin_make build type here, make available in c++ code for preprocessor.
# This allows usage of #IF DEBUG in all code of this package. It retrieves the value from cmakes CMAKE_BUILD_TYPE. 
# This can be switched using catkin_make -DCMAKE_BUILD_TYPE=Release , or catkin_make -DCMAKE_BUILD_TYPE=Debug
add_definitions(-DCMAKE_BUILD_TYPE_DEF=${CMAKE_BUILD_TYPE}) 
if(CMAKE_BUILD_TYPE MATCHES Debug)
add_definitions(-DDEBUG=1) 
else()
add_definitions(-DDEBUG=0)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  st_topological_mapping
  tf
)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(save_map src/save_map.cpp)
add_dependencies(save_map ${catkin_EXPORTED_TARGETS}) #to make sure  message headers are generated first!
target_link_libraries(save_map ${catkin_LIBRARIES} yaml-cpp)
