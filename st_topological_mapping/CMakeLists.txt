cmake_minimum_required(VERSION 2.8.3)
project(st_topological_mapping)

add_definitions(-DDEBUG=1 ) #This allows usage of #IF DEBUG in all code

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  tf
)

add_message_files(
  FILES
  TopologicalNavigationMap.msg
  TopoNavEdge.msg
  TopoNavNode.msg
)

generate_messages(   
  DEPENDENCIES  
  std_msgs 
  geometry_msgs
)  

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(topological_navigation_mapper src/utils.cpp src/toponav_node.cpp src/toponav_edge.cpp src/show_toponav_map.cpp src/toponav_map.cpp src/main.cpp)
target_link_libraries(topological_navigation_mapper ${catkin_LIBRARIES})
add_dependencies(topological_navigation_mapper ${catkin_EXPORTED_TARGETS}) # KL: to make sure that message headers in devel/include/${PROJECT_NAME} are recognized

install(
    TARGETS
        topological_navigation_mapper
	#add more lines for more nodes
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

