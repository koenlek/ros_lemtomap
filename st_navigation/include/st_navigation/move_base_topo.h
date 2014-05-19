/**
 * @file move_base_topo
 * @brief Navigate a robot based on a topological map and topological goal
 * @author Koen Lekkerkerker
 */

#ifndef MOVE_BASE_TOPO_H
#define MOVE_BASE_TOPO_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <st_navigation/shortest_paths.h>
#include <st_navigation/GotoNodeAction.h>

#include "st_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "st_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "st_topological_mapping/TopoNavNodeMsg.h"  //Message

class MoveBaseTopo
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<st_navigation::GotoNodeAction> action_server_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  st_navigation::GotoNodeFeedback feedback_;
  st_navigation::GotoNodeResult result_;

  st_topological_mapping::TopologicalNavigationMap toponavmap_;

  ros::Subscriber toponavmap_sub_;
  std::string toponav_map_topic_;

public:
  MoveBaseTopo(std::string name);
  ~MoveBaseTopo();
  void executeCB(const st_navigation::GotoNodeGoalConstPtr& goal);
  void toponavmapCB(const st_topological_mapping::TopologicalNavigationMapConstPtr& toponav_map);
  std::vector<int> nodesPathToEdgesPath(const std::vector<int>& path_nodes);
};

#endif
