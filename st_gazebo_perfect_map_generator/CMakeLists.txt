cmake_minimum_required(VERSION 2.8.3)
project(st_gazebo_perfect_map_generator)

find_package(Boost REQUIRED COMPONENTS system)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/gazebo_msgs
  )
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/gazebo_msgs)
add_subdirectory(gazebo_msgs)

find_package(
catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable (generate_map_files src/generate_map_files.cpp)
target_link_libraries(generate_map_files collision_map_creator_msgs ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(generate_map_files collision_map_creator_msgs)

add_library(collision_map_creator src/collision_map_creator.cpp)
target_link_libraries(collision_map_creator collision_map_creator_msgs ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(collision_map_creator collision_map_creator_msgs)
