/**
 * @file move_base_topo
 * @brief Navigate a robot based on a topological map and topological goal
 * @author Koen Lekkerkerker
 */

/**
 The move_base_topo ros node is an action server that can receive a topological node as a goal.
 It is actually *not* a nav_core global_planner. It only generates goals to pass to the global_planner through
 the move_base system. So move_base_topo generates a topological path (which has a similar role as the normal global plan),
 and passes a series of goals to move_base, which then uses the global and local planner to execute it.
 */

#include <lemto_navigation/move_base_topo.h>

MoveBaseTopo::MoveBaseTopo(std::string name) :
    action_server_mbt_(nh_, name, boost::bind(&MoveBaseTopo::executeCB, this, _1), false), action_name_mbt_(name), move_base_client_("move_base", true)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("main_loop_frequency", frequency_, 1.0); //requires a full path to the top level directory, ~ and other env. vars are not accepted

  toponav_map_topic_ = "topological_navigation_mapper/topological_navigation_map";
  toponavmap_sub_ = nh_.subscribe(toponav_map_topic_, 1, &MoveBaseTopo::toponavmapCB, this);
  asso_node_servcli_ = nh_.serviceClient<lemto_topological_mapping::GetAssociatedNode>("topological_navigation_mapper/get_associated_node");
  predecessor_map_servcli_ = nh_.serviceClient<lemto_topological_mapping::GetPredecessorMap>("topological_navigation_mapper/get_predecessor_map");
  directnav_servcli_ = nh_.serviceClient<lemto_topological_mapping::IsDirectNavigable>("topological_navigation_mapper/is_direct_navigable");

#if BENCHMARKING
  benchmark_inprogress_ = false;
#endif

  action_server_mbt_.start();

  ROS_INFO("Waiting for move_base action server");
  move_base_client_.waitForServer();
  ROS_INFO("Waiting for move_base action server -- Finished");
}

void MoveBaseTopo::toponavmapCB(const lemto_topological_mapping::TopologicalNavigationMapConstPtr& toponav_map)
    {
  ROS_DEBUG("Received map with %lu nodes and %lu edges", toponav_map->nodes.size(), toponav_map->edges.size());
  toponavmap_ = *toponav_map;
}

void MoveBaseTopo::executeCB(const lemto_actions::GotoNodeGoalConstPtr& goal) //CB stands for CallBack...
    {
#if BENCHMARKING
  benchmark_inprogress_ = true;
  cpuwatch_.restart(); //sets current `lap` to zero.
  stopwatch_.restart(); //sets current `lap` to zero.
  ROS_INFO("Received Topo goal at %.4f", ros::WallTime::now().toSec());
#endif

  // helper variables
  bool success = false;
  std::vector<int> path_nodes;
  std::vector<std::string> path_edges;
  int start_node_id;

  // Calculate the topological path
  start_node_id = getAssociatedNode();

  if (!getShortestPath(start_node_id, goal->target_node_id, path_nodes))
      {
    // if it has NOT found a valid shortest path
    ROS_WARN("No valid path could be found from Node %d to %d. This move_base_topo action is aborted", start_node_id, goal->target_node_id);
    result_.success = false;
    action_server_mbt_.setAborted(result_);
  }
  else
  { // if it has found a valid shortest path
    path_edges = nodesPathToEdgesPath(path_nodes);

    // Action server feedback
    feedback_.route_node_ids = path_nodes;
    feedback_.route_edge_ids = path_edges;
    action_server_mbt_.publishFeedback(feedback_);

    // Variables for generating move base goals
    move_base_msgs::MoveBaseGoal move_base_goal;
    tf::Pose robot_pose = getRobotPoseInTopoFrame();
    geometry_msgs::PoseStamped node_pose, node_pose_in_map_now, move_base_goal_pose_as_sent;
    node_pose.pose.orientation.w = 1.0; //position x,y,z default to 0 for now...
    node_pose.header.frame_id = toponavmap_.header.frame_id;

    // Generate mapping to find nodes by node id from toponavmsg
    std::map<int, int> nodes_id2vecpos_map;
    for (int i = 0; i < toponavmap_.nodes.size(); i++) {
      nodes_id2vecpos_map[toponavmap_.nodes.at(i).node_id] = i;
    }

    // start executing the action
    int i = 0;
    ros::Rate r(frequency_);
    while (!success && ros::ok()) //you need to add ros::ok(), otherwise the loop will never finish
    {
      // check if a /move_base_topo preempt has been requested -> a preempt (cancellation) will automatically be triggered if a new goal is sent!
      if (action_server_mbt_.isPreemptRequested() || !ros::ok())
          {
        ROS_INFO("%s: Preempted", action_name_mbt_.c_str());
        // set the action state to preempted
        result_.success = false;
        action_server_mbt_.setPreempted(result_);
        break;
      }

      // pass new goal node if within a certain distance of current goal node, or if it is the first goal. Only check for passing new goals if last is not passed already
      //ROS_INFO("getAssociatedNode()=%d, i=%d, path_nodes.at(i)=%d",getAssociatedNode(), i, path_nodes.at(i));
      if ((i == 0 || calcDistance(node_pose.pose, getRobotPoseInTopoFrame()) < 1.0) && i != path_nodes.size()) {
        //if (i == 0 || getAssociatedNode() == path_nodes.at(i) ) {
        //KL: commented directNavigable check out, not needed if dist_tolerance_intermediate<=1.
        //if (i == 0 || directNavigable(node_pose.pose.position, getRobotPoseInTopoFrame().getOrigin(), true)) {
        ROS_DEBUG("Navigating to goal node #%d, with NodeID %d", i + 1, path_nodes.at(i));
        node_pose.pose.position = toponavmap_.nodes.at(nodes_id2vecpos_map[path_nodes.at(i)]).pose.position;
        move_base_goal.target_pose = node_pose;
        move_base_client_.sendGoal(move_base_goal); //move_base_client can handle goals sent in different frames, upon receiving, it is once transformed (but not updated if transform between frames changes later).
        move_base_goal_pose_as_sent = poseTopNavMap2Map(move_base_goal.target_pose);

        i++; // current i is always +1 compared to the current goal node vector position in path_nodes
        //} //closes directNav if
      }

      // handle the occasion that move_base is aborted: e.g. if the robot is stuck even after /move_base recovery behavior.
      if (move_base_client_.getState() == actionlib::SimpleClientGoalState::ABORTED)
          {
        ROS_WARN("/move_base was Aborted. The robot is probably stuck, even after executing all /move_base recovery behaviors");
        result_.success = false;
        ROS_WARN("%s: Aborted", action_name_mbt_.c_str());
        action_server_mbt_.setAborted(result_);
        break;
      }

      // If the transform between map and toponav_map has changed significantly, resend the goal
      double update_dist = 0.3;
      node_pose_in_map_now = poseTopNavMap2Map(node_pose);
      if (calcDistance(move_base_goal_pose_as_sent.pose, node_pose_in_map_now.pose) > update_dist && move_base_client_.getState() != actionlib::SimpleClientGoalState::PREEMPTED) { // check for preemted: otherwise, there is a change that a normal move_base goal (e.g. 2D Nav Goal RVIZ) is launched, and shortly after that overruled again by this...
        ROS_INFO("Diff is > %.2fm, resending goal!", update_dist);
        move_base_goal.target_pose = node_pose_in_map_now;
        move_base_client_.sendGoal(move_base_goal);
        move_base_goal_pose_as_sent = poseTopNavMap2Map(move_base_goal.target_pose);
      }

      // Print some debugging info and sleep for a while to avoid wasting processing power to executing useless loops
      ROS_DEBUG("Distance between robot and intermediate Goal NodeID %d = %f", path_nodes.at(i - 1), calcDistance(node_pose.pose, getRobotPoseInTopoFrame()));
      r.sleep();

      // handle the occasion that move_base is preempted: e.g. if a user passes a simple 2d nav goal using RVIZ.
      if (move_base_client_.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
          {
        ROS_INFO("/move_base was Preempted. Probably a simple 2D nav goal has been sent manually through RVIZ. The Topo Nav Goal is now cancelled");
        result_.success = false;
        ROS_WARN("%s: Aborted", action_name_mbt_.c_str());
        action_server_mbt_.setAborted(result_);
        break;
      }

      // check if the final goal has been reached
      if (i == path_nodes.size() && move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
        success = true;
        ROS_INFO("Hooray: the final topological goal has been reached");
#if BENCHMARKING
        ROS_INFO("Reached Topo goal at %.4f", ros::WallTime::now().toSec());
        ROS_INFO_STREAM("!!!!! It took: " << cpuwatch_.elapsed() <<"[s]CPU from initial move_base_topo goal received until the goal was completed");
        ROS_INFO_STREAM("!!!!! It took: " << stopwatch_.elapsed() <<"[s]WALL from initial move_base_topo goal received until the goal was completed");
#endif
      }
    }

    // Handle a successful execution
    if (success)
    {
      result_.success = true;
      ROS_INFO("%s: Succeeded", action_name_mbt_.c_str());
      // set the action state to succeeded
      action_server_mbt_.setSucceeded(result_);
    }

    // clear the feedback -> this will also update rviz to not show the path anymore...
    feedback_.route_node_ids.clear();
    feedback_.route_edge_ids.clear();
    action_server_mbt_.publishFeedback(feedback_);
  }
}

/*!
 * \brief getCurrentPose
 */
bool MoveBaseTopo::getShortestPath(const int start_node_id, const int target_node_id, std::vector<int> &path_nodes)
    {
#if BENCHMARKING
  ecl::CpuWatch cpuwatch_path_calc;
  cpuwatch_path_calc.restart();
#endif

  // request Predecessor map through service
  lemto_topological_mapping::GetPredecessorMap srv;
  srv.request.source_node_id = start_node_id;

  if (!predecessor_map_servcli_.call(srv)) {
    ROS_INFO("Did not receive a valid response from get_predecessor_map service");
    return false;
  }

  TopoNavNode::PredecessorMapNodeID predecessor_map;
  for (int i = 0; i < srv.response.nodes.size(); i++) {
    predecessor_map[srv.response.nodes.at(i)] = srv.response.predecessors.at(i);
  }

  // use predecessor map to construct path
  path_nodes.push_back(target_node_id);
  int tmp_node_id = target_node_id;

  while (tmp_node_id != start_node_id) {
    tmp_node_id = predecessor_map[tmp_node_id];
    path_nodes.insert(path_nodes.begin(), tmp_node_id);
  }
#if BENCHMARKING
  ROS_INFO_STREAM("!!!!! It took: " << cpuwatch_path_calc.elapsed() <<"[s]CPU to find the topological path (in getShortestPath function only)");
  if (benchmark_inprogress_) {
    ecl::TimeStamp cpu_time, wall_time;
    cpu_time = cpuwatch_.split();
    wall_time = stopwatch_.split();
    ROS_INFO_STREAM("!!!!! It took: " << cpu_time << "[s]CPU from receiving move_base_topo goal (/move_base_topo/goal) to having a topological plan");
    ROS_INFO_STREAM("!!!!! It took: " << wall_time << "[s]WALL from receiving move_base_topo goal (/move_base_topo/goal) to having a topological plan");
    benchmark_inprogress_ = false;
    }
#endif

  //turn path_node_id_vector into a string and print it
  std::stringstream path_stringstream;
  std::string path_string;
  std::copy(path_nodes.begin(), path_nodes.end(), std::ostream_iterator<int>(path_stringstream, ", "));
  path_string = path_stringstream.str();
  path_string = path_string.substr(0, path_string.size() - 2);
  ROS_INFO("\nCalculated topological path.\nNode IDs: [%s]", path_string.c_str());

  return true;
}

/*!
 * \brief getCurrentPose
 */
tf::Pose MoveBaseTopo::getRobotPoseInTopoFrame()
{
  tf::Pose robot_pose_tf;
  tf::StampedTransform robot_transform_tf;

  try
  {
    tf_listener_.waitForTransform("toponav_map", "base_link", ros::Time(0), ros::Duration(10));
    tf_listener_.lookupTransform("toponav_map", "base_link", ros::Time(0), robot_transform_tf);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Error looking up transformation\n%s", ex.what());
  }

  robot_pose_tf.setOrigin(robot_transform_tf.getOrigin());
  robot_pose_tf.setRotation(robot_transform_tf.getRotation());

  return robot_pose_tf;
}

int MoveBaseTopo::getAssociatedNode()
{
  lemto_topological_mapping::GetAssociatedNode srv;
  if (asso_node_servcli_.call(srv)) {
    ROS_DEBUG("Received AssoNode: %d", (int )srv.response.asso_node_id);
    return (int) srv.response.asso_node_id;
  }
  else {
    ROS_ERROR("Failed to call service get_associated_node");
    return -1;
  }
}

std::vector<std::string> MoveBaseTopo::nodesPathToEdgesPath(const std::vector<int>& path_nodes)
    {
  std::vector<std::string> path_edges;

  for (int i = 0; i < path_nodes.size() - 1; i++)
      {
    for (int j = 0; j < toponavmap_.edges.size(); j++)
        {
      if ((toponavmap_.edges.at(j).start_node_id == path_nodes.at(i) && toponavmap_.edges.at(j).end_node_id == path_nodes.at(i + 1)) || (toponavmap_.edges.at(j).end_node_id == path_nodes.at(i) && toponavmap_.edges.at(j).start_node_id == path_nodes.at(i + 1)))
          {
        path_edges.push_back(toponavmap_.edges.at(j).edge_id);
        break;
      }
    }
  }
  return path_edges;
}

geometry_msgs::PoseStamped MoveBaseTopo::poseTopNavMap2Map(const geometry_msgs::PoseStamped& pose_in_toponav_map)
{
  geometry_msgs::PoseStamped pose_in_map;
  try
  {
    tf_listener_.waitForTransform("toponav_map", "map", ros::Time(0), ros::Duration(10));
    tf_listener_.transformPose("map", pose_in_toponav_map, pose_in_map);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Error looking up transformation\n%s", ex.what());
  }
  return pose_in_map;
}


const bool MoveBaseTopo::directNavigable(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2, bool global) {
  // request Predecessor map through service
  lemto_topological_mapping::IsDirectNavigable srv;
  int max_allowable_cost = 99;

  srv.request.start_point = point1;
  srv.request.end_point = point2;
  srv.request.global = global;
  srv.request.max_allowable_cost = max_allowable_cost;

  if (!directnav_servcli_.call(srv)) {
    ROS_INFO("Did not receive a valid response from is_direct_navigable service");
    return false;
  }

  ROS_DEBUG("Direct Navigable (service from move base topo) = %s", srv.response.direct_navigable ? "true" : "false");

  return srv.response.direct_navigable;
}

const bool MoveBaseTopo::directNavigable(const geometry_msgs::Point &point1, const tf::Point &point2, bool global) {
  geometry_msgs::Point point2_gm;
  pointTFToMsg(point2, point2_gm);

  return directNavigable(point1, point2_gm, global);
}

const bool MoveBaseTopo::directNavigable(const tf::Point &point1, const tf::Point &point2, bool global) {
  geometry_msgs::Point point1_gm, point2_gm;
  pointTFToMsg(point1, point1_gm);
  pointTFToMsg(point2, point2_gm);

  return directNavigable(point1_gm, point2_gm, global);
}

/*!
 * \brief Main
 */
int main(int argc, char** argv)
    {
  ros::init(argc, argv, "move_base_topo");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  double frequency; //main loop frequency in Hz

  private_nh.param("main_loop_frequency", frequency, 1.0); //requires a full path to the top level directory, ~ and other env. vars are not accepted

  ros::Rate r(frequency);

  MoveBaseTopo move_base_topo(ros::this_node::getName());

#ifdef CMAKE_BUILD_TYPE_DEF
#include <boost/preprocessor/stringize.hpp>
  if (BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF) != "Release")
    ROS_WARN("This node (%s) had CMAKE_BUILD_TYPE=%s. Please use catkin_make -DCMAKE_BUILD_TYPE=Release for benchmarks!", ros::this_node::getName().c_str(), BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF));
  else
    ROS_INFO("This node (%s) had CMAKE_BUILD_TYPE=%s.", ros::this_node::getName().c_str(), BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF));
#endif

  //Set default logger level for this ROS Node...
  /*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
   }*/

  while (n.ok())
  {

    ros::spinOnce();
    r.sleep();
    if (r.cycleTime() > ros::Duration(1 / frequency))
      ROS_WARN("%s main loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds (%.4fHz)", ros::this_node::getName().c_str(), frequency, r.cycleTime().toSec(), 1 / r.cycleTime().toSec());
  }

  return 0;
}
