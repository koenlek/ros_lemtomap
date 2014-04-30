#ifndef LOAD_MAP_H
#define LOAD_MAP_H

// General includes
#include "string"
#include <cstdio>
#include "boost/foreach.hpp"
#include <boost/filesystem.hpp>

// ROS includes
#include "ros/ros.h"
#include "ros/console.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"

// Semi-local includes (other metapackage packages)
#include "st_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "st_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "st_topological_mapping/TopoNavNodeMsg.h"  //Message

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

  /**
   * Public Methods
   */
  st_topological_mapping::TopologicalNavigationMap getTopologicalNavigationMapMsg() const { return toponav_map_readmsg_; }

private:
  st_topological_mapping::TopologicalNavigationMap toponav_map_readmsg_;
};

#endif
