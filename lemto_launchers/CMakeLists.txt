cmake_minimum_required(VERSION 2.8.3)
project(lemto_launchers)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_package(
)

include_directories(
	${catkin_INCLUDE_DIRS}
)
