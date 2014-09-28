#ifndef SAVE_MAP_H
#define SAVE_MAP_H

// General includes
#include "string"
#include <cstdio>
#include "boost/foreach.hpp"
#include <boost/filesystem.hpp> //KL: for creating the save dirs if necessary
//#include <algorithm> //std::find
#include "yaml-cpp/yaml.h"

// ROS includes
#include "ros/ros.h"
#include "ros/console.h"
#include "rosbag/bag.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include <actionlib/client/simple_action_client.h>

// Semi-local includes (other metapackage packages)
#include "lemto_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "lemto_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "lemto_topological_mapping/TopoNavNodeMsg.h"  //Message
#include "lemto_topological_mapping/GetAssociatedNode.h"  //Service

#include "lemto_topological_mapping/toponav_node.h"

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
  tf::TransformListener tf_listener_;
  ros::ServiceClient asso_node_servcli_;

  bool no_associated_node_and_no_pose_;

  geometry_msgs::Pose getRobotPose();
  int getAssociatedNode();
  /**
   * Methods
   */
  void toponavmapCB(lemto_topological_mapping::TopologicalNavigationMap toponav_map); //?copy instead of reference assures no problems of changes during saving?
};

YAML::Emitter& operator <<(YAML::Emitter& out, const geometry_msgs::Pose& pose);

#endif
