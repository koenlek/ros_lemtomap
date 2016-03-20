/**
 * @file move_base_topo
 * @brief Navigate a robot based on a topological map and topological goal
 * @author Koen Lekkerkerker
 */

#ifndef MOVE_BASE_TOPO_H
#define MOVE_BASE_TOPO_H

#ifndef BENCHMARKING
#define BENCHMARKING 0
#endif

// ROS includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>

#if BENCHMARKING
#include <ecl/time/stopwatch.hpp>
#include <ecl/time/cpuwatch.hpp>
#include <ecl/time/time_data.hpp>
#endif

// Local includes
#include "lemto_topological_mapping/utils.h" //calcDistance

#include "lemto_actions/GotoNodeAction.h" //Action
#include "lemto_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "lemto_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "lemto_topological_mapping/TopoNavNodeMsg.h"  //Message
#include "lemto_topological_mapping/GetAssociatedNode.h"  //Service
#include "lemto_topological_mapping/GetPredecessorMap.h"  //Service
#include "lemto_topological_mapping/IsDirectNavigable.h"  //Service


class MoveBaseTopo
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
  actionlib::SimpleActionServer<lemto_actions::GotoNodeAction> action_server_mbt_; //mbt -> move_base_topo
  std::string action_name_mbt_;
  // create messages that are used to published feedback/result
  lemto_actions::GotoNodeFeedback feedback_;
  lemto_actions::GotoNodeResult result_;

  double frequency_; //main loop frequency in Hz

  lemto_topological_mapping::TopologicalNavigationMap toponavmap_;

  ros::Subscriber toponavmap_sub_;
  ros::ServiceClient asso_node_servcli_;
  ros::ServiceClient predecessor_map_servcli_;
  ros::ServiceClient directnav_servcli_;

  std::string toponav_map_topic_;
  std::string goal_frame_id_;
  tf::TransformListener tf_listener_;

#if BENCHMARKING
  ros::Subscriber move_base_global_plan_sub_;
  void moveBaseGlobalPlanCB(const nav_msgs::PathConstPtr& path);
  ecl::CpuWatch cpuwatch_;
  ecl::StopWatch stopwatch_;
  bool benchmark_inprogress_;
#endif

public:
  MoveBaseTopo(std::string name);

private:
  tf::Pose getRobotPoseInTopoFrame();
  int getAssociatedNode();
  void executeCB(const lemto_actions::GotoNodeGoalConstPtr& goal);
  void toponavmapCB(const lemto_topological_mapping::TopologicalNavigationMapConstPtr& toponav_map);
  std::vector<std::string> nodesPathToEdgesPath(const std::vector<int>& path_nodes);
  bool getShortestPath(const int start_node_id, const int target_node_id, std::vector<int> &path_nodes);
  geometry_msgs::PoseStamped poseTopNavMap2Map(const geometry_msgs::PoseStamped& pose_in_toponav_map);
  const bool directNavigable(const tf::Point &point1, const tf::Point &point2, bool global);
  const bool directNavigable(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2, bool global);
  const bool directNavigable(const geometry_msgs::Point &point1, const tf::Point &point2, bool global);


};

#endif
