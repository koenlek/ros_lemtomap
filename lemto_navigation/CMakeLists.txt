cmake_minimum_required(VERSION 2.8.3)
project(lemto_navigation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_definitions(-DBENCHMARKING=1) 

if(BENCHMARKING MATCHES 1)
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO) #compiles out all ROS_DEBUG logging, which gives extra speed as it does not need to be printed/processed/logged to disk. Choose from DEBUG INFO WARN ERROR FATAL and NONE (none means no logging at all)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  message_generation
  #std_msgs
  #move_base_msgs 
  tf
  lemto_topological_mapping
  ecl_time
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

add_executable(move_base_topo src/move_base_topo.cpp)
target_link_libraries(move_base_topo ${catkin_LIBRARIES})
add_dependencies(move_base_topo lemto_topological_mapping_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
