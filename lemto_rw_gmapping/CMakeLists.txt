cmake_minimum_required(VERSION 2.8)
project(lemto_rw_gmapping)

# for std::vector::shrink_to_fit
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_definitions(-DBENCHMARKING=1) 

if(BENCHMARKING MATCHES 1)
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO) #compiles out all ROS_DEBUG logging, which gives extra speed as it does not need to be printed/processed/logged to disk. Choose from DEBUG INFO WARN ERROR FATAL and NONE (none means no logging at all)
endif()

find_package(catkin REQUIRED nav_msgs openslam_rw_gmapping roscpp rostest tf)

find_package(Boost REQUIRED signals)

include_directories(${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(src)

catkin_package()

add_executable(slam_rw_gmapping src/rw_gmapping.cpp src/main.cpp)
target_link_libraries(slam_rw_gmapping ${BOOST_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(slam_rw_gmapping ${catkin_EXPORTED_TARGETS})
endif()
