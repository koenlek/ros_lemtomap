/**
 * @file main.cpp
 * @brief This file contains the main function that forms the topological_navigation_mapper ROS node.
 * @author Koen Lekkerkerker
 */

#include <lemto_topological_mapping/toponav_map.h>
#include <lemto_topological_mapping/show_toponav_map.h>
#include <lemto_topological_mapping/load_map.h>

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
  bool disable_visualizations;
  ShowTopoNavMap* show_topo_nav_map;

  private_nh.param("load_map_directory", load_map_path, std::string("")); //requires a full path to the top level directory, ~ and other env. vars are not accepted
  private_nh.param("main_loop_frequency", frequency, double(4.0));
  private_nh.param("disable_visualizations", disable_visualizations, bool(false)); //disables all visualizations

  ros::Rate r(frequency);

  TopoNavMap topo_nav_map(n);

  if (load_map_path != "") {
    ROS_INFO("Map will be loaded from: %s", load_map_path.c_str());
    StMapLoader map_loader(load_map_path);
    while (!map_loader.finished_loading_) {
      ros::spinOnce();
    }
    topo_nav_map.loadSavedMap(map_loader.toponav_map_readmsg_,
                              map_loader.associated_node_,
                              map_loader.robot_pose_);
  }

  if (!disable_visualizations)
    show_topo_nav_map = new ShowTopoNavMap(n, topo_nav_map.getNodes(), topo_nav_map.getEdges(), topo_nav_map.getAssociatedNode(),frequency);

#ifdef CMAKE_BUILD_TYPE_DEF
#include <boost/preprocessor/stringize.hpp>
  if (BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF) != "Release")
    ROS_WARN("This node (%s) had CMAKE_BUILD_TYPE=%s. Please use catkin_make -DCMAKE_BUILD_TYPE=Release for benchmarks!", ros::this_node::getName().c_str(), BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF));
  else
    ROS_INFO("This node (%s) had CMAKE_BUILD_TYPE=%s.", ros::this_node::getName().c_str(), BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF));
#endif


  while (n.ok()) {

    topo_nav_map.updateMap();
    if (!disable_visualizations)
      show_topo_nav_map->updateVisualization();

    ros::spinOnce();
    r.sleep();
    if (r.cycleTime() > ros::Duration(1 / frequency))
      ROS_WARN("%s main loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds (%.4fHz)", ros::this_node::getName().c_str(), frequency,
               r.cycleTime().toSec(),
               1 / r.cycleTime().toSec());
  }

  if (show_topo_nav_map)
    delete show_topo_nav_map;
  return 0;
}
