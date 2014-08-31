cmake_minimum_required(VERSION 2.8.3)
project(st_tools)

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

if(BENCHMARKING MATCHES 1)
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO) #compiles out all ROS_DEBUG logging, which gives extra speed as it does not need to be printed/processed/logged to disk. Choose from DEBUG INFO WARN ERROR FATAL and NONE (none means no logging at all)
endif()

find_package(
catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_logger src/odom_logger.cpp)
target_link_libraries(odom_logger ${catkin_LIBRARIES})
add_dependencies(odom_logger ${catkin_EXPORTED_TARGETS})

add_executable(cmd_vel_logger src/cmd_vel_logger.cpp)
target_link_libraries(cmd_vel_logger ${catkin_LIBRARIES})
add_dependencies(cmd_vel_logger ${catkin_EXPORTED_TARGETS})

