/**
 * @file main.cpp
 * @brief This file contains the main function that forms the topological_navigation_mapper ROS node.
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/toponav_map.h>
#include <st_topological_mapping/show_toponav_map.h>
#include <st_topological_mapping/load_map.h>

/*!
 * Main
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "topological_navigation_mapper");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  std::string load_map_path;
  double frequency; //main loop frequency in Hz

  private_nh.param("load_map_directory", load_map_path, std::string("")); //requires a full path to the top level directory, ~ and other env. vars are not accepted
  private_nh.param("main_loop_frequency", frequency, 4.0); //requires a full path to the top level directory, ~ and other env. vars are not accepted

  ros::Rate r(frequency);

  TopoNavMap topo_nav_map(n);
#if LTF_PERFECTODOM
  if (load_map_path != ""){
  ROS_ERROR("Loading map while using 'LTF_PERFECTODOM 1' is impossible, skipping map loading...");
  load_map_path="";
  }
#endif
  if (load_map_path != "") {
    ROS_INFO("Map will be loaded from: %s", load_map_path.c_str());
    StMapLoader map_loader(load_map_path);
    while (!map_loader.finished_loading_) {
      ros::spinOnce();
    }
    topo_nav_map.loadMapFromMsg(
                                map_loader.getTopologicalNavigationMapMsg());
  }

  ShowTopoNavMap show_topo_nav_map(n, topo_nav_map.getNodes(), topo_nav_map.getEdges(), topo_nav_map.getAssociatedNode());

#ifdef CMAKE_BUILD_TYPE_DEF
#include <boost/preprocessor/stringize.hpp>
  if (BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF) != "Release")
    ROS_WARN("This node (%s) had CMAKE_BUILD_TYPE=%s. Please use catkin_make -DCMAKE_BUILD_TYPE=Release for benchmarks!", ros::this_node::getName().c_str(), BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF));
  else
    ROS_INFO("This node (%s) had CMAKE_BUILD_TYPE=%s.", ros::this_node::getName().c_str(), BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF));
#endif

  /*int cycle = 0, cycle_period = 0;
   ros::Time period(1 * 60); //1 minute
   double freq_avg, freq_cur, freq_avg_period;
   std::vector<ros::Time> benchm_update_time;
   benchm_update_time.push_back(ros::Time::now());*/

  ecl::CpuWatch cpuwatch;
  ecl::TimeData timings;
  ecl::Duration duration;

  while (n.ok()) {
    cpuwatch.restart();

    /*cycle++;
     benchm_update_time.push_back(ros::Time::now());
     freq_cur=1/(benchm_update_time.at(cycle)-benchm_update_time.at(cycle-1)).toSec();
     freq_avg=cycle/(benchm_update_time.at(cycle)-benchm_update_time.at(0)).toSec();
     ROS_INFO("Current Time: %f",ros::Time::now().toSec());
     ROS_INFO("Main cycle duration current=%.2f[Hz] avg=%.2f[Hz]",freq_cur,freq_avg);*/

    topo_nav_map.updateMap();
    show_topo_nav_map.updateVisualization();

    duration = cpuwatch.split();
    timings.push_back(duration);

    ROS_DEBUG_STREAM("Average : " << timings.average());
    ROS_DEBUG_STREAM("Variance: " << timings.variance());

    ros::spinOnce();
    r.sleep();
    if (r.cycleTime() > ros::Duration(1 / frequency))
      ROS_WARN("%s main loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds (%.4fHz)", ros::this_node::getName().c_str(), frequency,
               r.cycleTime().toSec(),
               1 / r.cycleTime().toSec());
  }

  return 0;
}
