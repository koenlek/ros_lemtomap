#ifndef LOAD_MAP_H
#define LOAD_MAP_H

// General includes
#include "string"
#include <cstdio>
#include "boost/foreach.hpp"
//#include <algorithm> //std::find

// ROS includes
#include "ros/ros.h"
#include "ros/console.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"

// Semi-local includes (other metapackage packages)
#include "st_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "st_topological_mapping/TopoNavEdge.h"  //Message
#include "st_topological_mapping/TopoNavNode.h"  //Message

class StMapLoader
{
public:

  StMapLoader(const std::string& mapname);

  /**
   * Variables
   */
  std::string mapname_;
  std::string toponav_map_topic_;
  bool finished_loading_;
};

#endif
