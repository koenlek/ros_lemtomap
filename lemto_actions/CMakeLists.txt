cmake_minimum_required(VERSION 2.8.3)
project(lemto_actions)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_definitions(-DBENCHMARKING=1) 

if(BENCHMARKING MATCHES 1)
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO) #compiles out all ROS_DEBUG logging, which gives extra speed as it does not need to be printed/processed/logged to disk. Choose from DEBUG INFO WARN ERROR FATAL and NONE (none means no logging at all)
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
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
  CATKIN_DEPENDS 
   actionlib 
   actionlib_msgs
   message_runtime
)