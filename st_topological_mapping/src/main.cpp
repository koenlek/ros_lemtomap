/**
 * @file main.cpp
 * @brief This file contains the main function that forms the topological_navigation_mapper ROS node.
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/toponav_map.h>
#include <st_topological_mapping/show_toponav_map.h>

/*!
 * Main
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "topological_navigation_mapper");
  ros::NodeHandle n;

  TopoNavMap topo_nav_map(n);
  ShowTopoNavMap show_topo_nav_map(n, topo_nav_map.getNodes(), topo_nav_map.getEdges());

  ros::Rate r(4);

  while (n.ok())
  {
    topo_nav_map.updateMap();
    show_topo_nav_map.updateVisualization();

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
