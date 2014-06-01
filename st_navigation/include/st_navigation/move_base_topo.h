/**
 * @file move_base_topo
 * @brief Navigate a robot based on a topological map and topological goal
 * @author Koen Lekkerkerker
 */

#ifndef MOVE_BASE_TOPO_H
#define MOVE_BASE_TOPO_H

// ROS includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>

#include <ecl/time/stopwatch.hpp>
#include <ecl/time/cpuwatch.hpp>
#include <ecl/time/time_data.hpp>

// Local includes
#include <st_navigation/shortest_paths.h>
#include <st_navigation/GotoNodeAction.h>

#include "st_topological_mapping/utils.h" //calcDistance
#include "st_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "st_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "st_topological_mapping/TopoNavNodeMsg.h"  //Message

class MoveBaseTopo
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
  actionlib::SimpleActionServer<st_navigation::GotoNodeAction> action_server_mbt_; //mbt -> move_base_topo
  std::string action_name_mbt_;
  // create messages that are used to published feedback/result
  st_navigation::GotoNodeFeedback feedback_;
  st_navigation::GotoNodeResult result_;

  st_topological_mapping::TopologicalNavigationMap toponavmap_;

  ros::Subscriber toponavmap_sub_;
  std::string toponav_map_topic_;
  std::string goal_frame_id_;
  tf::TransformListener tf_listener_;

#if BENCHMARKING
  ros::Subscriber move_base_feedback_sub_;
  void moveBaseGlobalPlanCB(const nav_msgs::PathConstPtr& path);
  ecl::CpuWatch cpuwatch_;
  ecl::StopWatch stopwatch_;
  bool benchmark_inprogress;
#endif

public:
  MoveBaseTopo(std::string name);

private:
  tf::Pose getCurrentPose();
  int getCurrentAssociatedNode();
  void executeCB(const st_navigation::GotoNodeGoalConstPtr& goal);
  void toponavmapCB(const st_topological_mapping::TopologicalNavigationMapConstPtr& toponav_map);
  std::vector<int> nodesPathToEdgesPath(const std::vector<int>& path_nodes);

};

#endif