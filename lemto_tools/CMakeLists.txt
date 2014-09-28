cmake_minimum_required(VERSION 2.8.3)
project(lemto_tools)

add_definitions(-DBENCHMARKING=1) 

if(BENCHMARKING MATCHES 1)
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO) #compiles out all ROS_DEBUG logging, which gives extra speed as it does not need to be printed/processed/logged to disk. Choose from DEBUG INFO WARN ERROR FATAL and NONE (none means no logging at all)
endif()

find_package(
catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
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

add_executable(time_logger src/time_logger.cpp)
target_link_libraries(time_logger ${catkin_LIBRARIES})
add_dependencies(time_logger ${catkin_EXPORTED_TARGETS})

add_executable(publish_perfect_odom_frame src/publish_perfect_odom_frame.cpp)
target_link_libraries(publish_perfect_odom_frame ${catkin_LIBRARIES})
add_dependencies(publish_perfect_odom_frame ${catkin_EXPORTED_TARGETS})


