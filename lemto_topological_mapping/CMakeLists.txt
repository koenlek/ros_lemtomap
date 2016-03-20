cmake_minimum_required(VERSION 2.8.3)
project(lemto_topological_mapping)

add_definitions(-DBENCHMARKING=1) 

if(BENCHMARKING MATCHES 1)
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO) #compiles out all ROS_DEBUG logging, which gives extra speed as it does not need to be printed/processed/logged to disk. Choose from DEBUG INFO WARN ERROR FATAL and NONE (none means no logging at all)
endif()

find_package(
catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  tf
  lemto_actions
  rosbag #for loading previous maps
  ecl_time
  cmake_modules #Indigo compatibility
)

find_package(Eigen REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  TopologicalNavigationMap.msg
  TopoNavEdgeMsg.msg
  TopoNavNodeMsg.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetAssociatedNode.srv
  GetPredecessorMap.srv
  IsDirectNavigable.srv
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
target_link_libraries(topological_navigation_mapper ${catkin_LIBRARIES} yaml-cpp) #yaml cpp for src/load_map.cpp
add_dependencies(topological_navigation_mapper ${catkin_EXPORTED_TARGETS}) # KL: to make sure that message headers in devel/include/${PROJECT_NAME} are recognized

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-unittest_example test/unittest_example.cpp)
if(TARGET ${PROJECT_NAME}-unittest_example)
  #target_link_libraries(${PROJECT_NAME}-unittest_example ${PROJECT_NAME})
endif()

