cmake_minimum_required(VERSION 2.8.3)
project(st_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  move_base_msgs 
  actionlib
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigate_waypoints src/navigate_waypoints.cpp)
target_link_libraries(navigate_waypoints ${catkin_LIBRARIES})
