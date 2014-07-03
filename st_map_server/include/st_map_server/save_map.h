#ifndef SAVE_MAP_H
#define SAVE_MAP_H

// General includes
#include "string"
#include <cstdio>
#include "boost/foreach.hpp"
#include <boost/filesystem.hpp> //KL: for creating the save dirs if necessary
//#include <algorithm> //std::find

// ROS includes
#include "ros/ros.h"
#include "ros/console.h"
#include "rosbag/bag.h"

// Semi-local includes (other metapackage packages)
#include "st_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "st_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "st_topological_mapping/TopoNavNodeMsg.h"  //Message

class StMapSaver
{
public:

  StMapSaver(const std::string& mapname);

  /**
   * Variables
   */
  std::string mapname_;
  std::string toponav_map_topic_;
  ros::Subscriber toponavmap_sub_;
  bool saved_map_;
  bool callback_started_;

  /**
   * Public Methods
   */
  void toponavmapCB(st_topological_mapping::TopologicalNavigationMap toponav_map); //?copy instead of reference assures no problems of changes during saving?
};

#endif
