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
	toponav_map_topic_ = "topological_navigation_mapper/topological_navigation_map";
	toponavmap_sub_ = nh_.subscribe(toponav_map_topic_, 1, &MoveBaseTopo::toponavmapCB, this);
	action_server_.start();
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
    bool success = true;
    std::vector<int> path_nodes;
    std::vector<int> path_edges;
    ros::Time start_time = ros::Time::now();

    // publish info to the console for the user
    ROS_INFO("%s: Executing, navigating to the stars and back, to finish at node_id %d", action_name_.c_str(), goal->target_node_id);

    // Calculate the topological path
    path_nodes = st_shortest_paths::shortestPath(toponavmap_,1,goal->target_node_id);

    // Action server feedback
    feedback_.route_node_ids=path_nodes;
    path_edges=nodesPathToEdgesPath(path_nodes);
    feedback_.route_edge_ids=path_edges;
    action_server_.publishFeedback(feedback_);

    // start executing the action
    while (ros::Time::now()<start_time+ros::Duration(10)) //TODO: this time limit is just for testing: should be removed eventually
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

std::vector<int> MoveBaseTopo::nodesPathToEdgesPath(const std::vector<int>& path_nodes)
{
	std::vector<int> path_edges;

	for (int i = 0; i < path_nodes.size()-1; i++)
	{
		for (int j = 0; j < toponavmap_.edges.size(); j++)
		{
			if((toponavmap_.edges.at(j).start_node_id==path_nodes.at(i) && toponavmap_.edges.at(j).end_node_id==path_nodes.at(i+1)) ||
					(toponavmap_.edges.at(j).end_node_id==path_nodes.at(i) && toponavmap_.edges.at(j).start_node_id==path_nodes.at(i+1)))
			{
				path_edges.push_back(toponavmap_.edges.at(j).edge_id);
				break;
			}
		}
	}
	return path_edges;
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

	//Set default logger level for this ROS Node...
	/*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}*/

	while (n.ok()) {

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
