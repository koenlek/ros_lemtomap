#ifndef LOAD_MAP_H
#define LOAD_MAP_H

// General includes
#include "string"
#include <cstdio>
#include "boost/foreach.hpp"
#include <boost/filesystem.hpp>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>

// ROS includes
#include "ros/ros.h"
#include "ros/console.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"

// Semi-local includes (other metapackage packages)
#include "lemto_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "lemto_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "lemto_topological_mapping/TopoNavNodeMsg.h"  //Message

class StMapLoader
{
public:

  StMapLoader(const std::string& map_fullpath);

  /**
   * Variables
   */
  std::string map_fullpath_;
  std::string toponav_map_topic_;
  bool finished_loading_;

  lemto_topological_mapping::TopologicalNavigationMap toponav_map_readmsg_;
  int associated_node_;
  geometry_msgs::Pose robot_pose_;
  /**
   * Methods
   */

  void loadBag();
  void loadYAML();

private:

};

void operator >> (const YAML::Node& node, geometry_msgs::Pose& robot_pose);

#endif
