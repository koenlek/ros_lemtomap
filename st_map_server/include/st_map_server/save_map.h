#ifndef SAVE_MAP_H
#define SAVE_MAP_H

// General includes
#include "string"
//#include <algorithm> //std::find

// ROS includes
#include "ros/ros.h"

// Semi-local includes (other metapackage packages)
#include "st_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "st_topological_mapping/TopoNavEdge.h"  //Message
#include "st_topological_mapping/TopoNavNode.h"  //Message

class StMapSaver
{
public:

  StMapSaver();

  /**
   * Variables
   */
  std::string mapname_;
  std::string toponav_map_topic_;
  ros::Subscriber toponavmap_sub_;
  bool saved_map_;

  /**
   * Public Methods
   */
  void toponavmapCallback(const st_topological_mapping::TopologicalNavigationMapPtr& toponav_map_ptr);
};

#endif
