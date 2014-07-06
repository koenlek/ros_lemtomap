cmake_minimum_required(VERSION 2.8)
project(st_gmapping_rolling)

# Load the used catkin_make build type here, make available in c++ code for preprocessor.
# This allows usage of #IF DEBUG in all code of this package. It retrieves the value from cmakes CMAKE_BUILD_TYPE. 
# This can be switched using catkin_make -DCMAKE_BUILD_TYPE=Release , or catkin_make -DCMAKE_BUILD_TYPE=Debug
add_definitions(-DCMAKE_BUILD_TYPE_DEF=${CMAKE_BUILD_TYPE}) 
if(CMAKE_BUILD_TYPE MATCHES Debug)
add_definitions(-DDEBUG=1) 
else()
add_definitions(-DDEBUG=0)
endif()

if(BENCHMARKING MATCHES 1)
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO) #compiles out all ROS_DEBUG logging, which gives extra speed as it does not need to be printed/processed/logged to disk. Choose from DEBUG INFO WARN ERROR FATAL and NONE (none means no logging at all)
endif()

find_package(catkin REQUIRED nav_msgs openslam_gmapping roscpp rostest tf)

find_package(Boost REQUIRED signals)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(src)

catkin_package()

add_executable(st_gmapping_rolling src/st_gmapping_rolling.cpp src/main.cpp)
target_link_libraries(st_gmapping_rolling ${Boost_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(st_gmapping_rolling ${catkin_EXPORTED_TARGETS})
endif()
