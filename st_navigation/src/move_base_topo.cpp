/**
 * @file move_base_topo
 * @brief Navigate a robot based on a topological map and topological goal
 * @author Koen Lekkerkerker
 */

#include <st_navigation/move_base_topo.h>

MoveBaseTopo::MoveBaseTopo(std::string name):
    action_server_(nh_, name, boost::bind(&MoveBaseTopo::executeCB, this, _1), false),
    action_name_(name)
  {
	toponav_map_topic_ =
				"topological_navigation_mapper/topological_navigation_map";
	action_server_.start();
	toponavmap_sub_ = nh_.subscribe(toponav_map_topic_, 1, &MoveBaseTopo::toponavmapCB, this);
  }

MoveBaseTopo::~MoveBaseTopo(void)
  {
  }

void MoveBaseTopo::toponavmapCB(const st_topological_mapping::TopologicalNavigationMapConstPtr& toponav_map)
{
		ROS_DEBUG("Received map with %lu nodes and %lu edges",
				toponav_map->nodes.size(), toponav_map->edges.size());
		toponavmap_=*toponav_map;
}

void MoveBaseTopo::executeCB(const st_navigation::GotoNodeGoalConstPtr& goal) //CB stands for CallBack...
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    ros::Time start_time = ros::Time::now();

    // publish info to the console for the user
    ROS_INFO("%s: Executing, navigating to the stars and back, to finish at node_id %d", action_name_.c_str(), goal->target_node_id);

    // Calculate the topological path
	st_shortest_paths::shortestPath(toponavmap_,1,goal->target_node_id);

    // start executing the action
    while (ros::Time::now()<start_time+ros::Duration(10))
    {
      // check that preempt has not been requested by the client -> a preempt (cancelation) will automatically be triggered if a new goal is send!
      if (action_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        result_.success = false;
        action_server_.setPreempted(result_);
        success = false;
        break;
      }
      feedback_.route_node_ids.push_back(1);
      feedback_.route_edge_ids.push_back(2);
      // publish the feedback
      action_server_.publishFeedback(feedback_);
    }

    if(success)
    {
      result_.success = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      action_server_.setSucceeded(result_);
    }
    else
    {
        result_.success = false;
    	ROS_INFO("%s: Failed", action_name_.c_str());
    	//do not set succeeded or abort here: it might be that a new action was requested and thus the action is still alive in an active status! I guess (KL)...
    }
  }

/*!
 * Main
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, "move_base_topo");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	ros::Rate r(4);

	MoveBaseTopo move_base_topo(ros::this_node::getName());

	while (n.ok()) {

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
