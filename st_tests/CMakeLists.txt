cmake_minimum_required(VERSION 2.8.3)
project(st_tests)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY rosbag/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbag
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
)

install(PROGRAMS
   scripts/timed_roslaunch.sh
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
