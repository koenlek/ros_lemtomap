/**
 * @file toponav_map
 * @brief Build and maintain the topological navigation map
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/toponav_map.h>

/*!
 * Constructor.
 */
TopoNavMap::TopoNavMap(ros::NodeHandle &n) :
		n_(n) // this way, TopoNavMap is aware of the NodeHandle of this ROS node, just as ShowTopoNavMap will be...
{
	ros::NodeHandle private_nh("~");

	// Parameters initialization
	private_nh.param("scan_topic", scan_topic_, std::string("scan"));

	scan_sub_ = n_.subscribe(scan_topic_, 10, &TopoNavMap::laserCallback, this);
	toponav_map_pub_ = private_nh.advertise<
			st_topological_mapping::TopologicalNavigationMap>(
			"topological_navigation_map", 1,true);

	updateMap(); //update the map one time, at construction. This will create the first map node.

#if DEBUG
	test_executed_ = 0;
#endif
}

TopoNavMap::~TopoNavMap() {
	while (edges_.size() > 0) {
		delete edges_.rbegin()->second;
	}
	while (nodes_.size() > 0) {
		delete nodes_.rbegin()->second;
	}
	std::cerr
			<< "~TopoNavMap: Deleting object -> all TopoNavNodes and TopoNavEdges are destructed"
			<< std::endl;
}

/*!
 * Laser Callback. Update laser_scan_
 */
void TopoNavMap::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
	ROS_DEBUG("LaserCallback");
	laser_scan_ = *msg;
	ROS_DEBUG("angle_max=%f", laser_scan_.angle_max); // to check whether it uses kinect vs hokuyo
}

/*!
 * Publish the Topological Navigation Map.
 */
void TopoNavMap::publishTopoNavMap() {
	ROS_DEBUG("publishTopoNavMap");
	st_topological_mapping::TopologicalNavigationMap msg_map;

	for (std::map<NodeID, TopoNavNode*>::iterator it=nodes_.begin(); it!=nodes_.end(); it++) {
		msg_map.nodes.push_back(nodeToRosMsg(it->second));
	}
	for (std::map<EdgeID, TopoNavEdge*>::iterator it=edges_.begin(); it!=edges_.end(); it++) {
		msg_map.edges.push_back(edgeToRosMsg(it->second));
	}

	toponav_map_pub_.publish(msg_map);
}

/*!
 * getCurrentPose
 */
void TopoNavMap::getCurrentPose() {
	try {
		tf_listener_.waitForTransform("/map", "/base_link", ros::Time(0),
				ros::Duration(10));
		tf_listener_.lookupTransform("/map", "/base_link", ros::Time(0),
				robot_transform_tf_);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("Error looking up transformation\n%s", ex.what());
	}

	robot_pose_tf_.setOrigin(robot_transform_tf_.getOrigin());
	robot_pose_tf_.setRotation(robot_transform_tf_.getRotation());

	ROS_DEBUG("Pose is x=%f, y=%f, theta=%f", robot_pose_tf_.getOrigin().x(),
			robot_pose_tf_.getOrigin().y(),
			tf::getYaw(robot_pose_tf_.getRotation()));
}

/*!
 * loadMapFromMsg
 */
void TopoNavMap::loadMapFromMsg(
		const st_topological_mapping::TopologicalNavigationMap &toponavmap_msg) {
	nodes_.clear();
	edges_.clear();

	for (int i = 0; i < toponavmap_msg.nodes.size(); i++) {
		nodeFromRosMsg(toponavmap_msg.nodes.at(i), nodes_);
		ROS_DEBUG("Loaded node with ID=%lu to std::map nodes_", toponavmap_msg.nodes.at(i).node_id);
	}
	for (int i = 0; i < toponavmap_msg.edges.size(); i++) {
		new TopoNavEdge(
				toponavmap_msg.edges.at(i).edge_id, //edge_id
				toponavmap_msg.edges.at(i).last_updated, //last_updated
				toponavmap_msg.edges.at(i).cost, //cost
				*nodes_[toponavmap_msg.edges.at(i).start_node_id],
				*nodes_[toponavmap_msg.edges.at(i).end_node_id],
				edges_ //edges std::map
				);
		ROS_DEBUG("Loaded edge with ID=%lu to std::map edges_", toponavmap_msg.nodes.at(i).node_id);
	}
	ROS_INFO("Finished loading the TopoNavMap");
}

/*!
 * updateMap
 */
void TopoNavMap::updateMap() {

	getCurrentPose();

	checkCreateNode();

	publishTopoNavMap();

#if DEBUG

	 /*if (ros::Time().now()>ros::Time(35) && test_executed_==0)
	 { //this code is to test stuff timed...
	 ROS_INFO("Deleting node 6");
	 deleteNode(6);
	 test_executed_++;
	 }
	 if (ros::Time().now()>ros::Time(37) && test_executed_==1)
	 { //this code is to test stuff timed...
	 ROS_INFO("Deleting node 2");
	 deleteNode(2);
	 test_executed_++;
	 }

	 if (ros::Time().now()>ros::Time(39) && test_executed_==2)
	 { //this code is to test stuff timed...
	 ROS_INFO("Moving node 4");
	 tf::Pose tmp_pose=nodes_[4]->getPose();
	 tmp_pose.getOrigin().setY(tmp_pose.getOrigin().getY()+0.3);
	 nodes_[4]->setPose(tmp_pose);
	 test_executed_++;
	 }*/

#endif
}

/*!
 * checkCreateNewNode
 */
bool TopoNavMap::checkCreateNode() {
	int number_of_nodes = getNumberOfNodes();
	int area_id = 1; //FIXME: room_id is always 1!
	bool create_node = false;
	bool is_door = false;

	if (checkIsNewDoor()) {
		//TODO: later, maybe door nodes should not influence other nodes. Maybe they should not be regular nodes at all. Check SAS10 for comparison.
		create_node = true;
		is_door = true;
	} else if (distanceToClosestNode() > 1) {
		//TODO FIXME: Remove magic number "1", which is the min distance here...
		create_node = true;
	}
	if (create_node) {
		addNode(robot_pose_tf_, is_door, area_id);
		checkCreateEdges((*nodes_.rbegin()->second)); //(*nodes_.rbegin()->second) should pass it the node that was just created...

		return true;
	} else {
		ROS_DEBUG("No new node created");
		return false;
	}
}

/*!
 * checkCreateEdges
 */
bool TopoNavMap::checkCreateEdges(const TopoNavNode &node) {
	//@TODO This method compares with all nodes: does not scale very well.
	bool edge_created = false;
	if (getNumberOfNodes() >= 2) {
		for (std::map<NodeID, TopoNavNode*>::iterator it=nodes_.begin(); it!=nodes_.end(); it++) {
			if (it->second->getNodeID() == node.getNodeID())
				continue; //Not compare with itself
			if (!edgeExists(node, *(it->second))
					&& directNavigable(node, *(it->second))) {
				addEdge(node, *(it->second));
				edge_created = true;
			}
		}
	}
	ROS_DEBUG_COND(!edge_created,
			"During this 'checkCreateEdges' call, no edge was created.");
	return edge_created;

}

/*!
 * directNavigable
 */
const bool TopoNavMap::directNavigable(const TopoNavNode &node1,
		const TopoNavNode &node2) const {
//@TODO write this method
	ROS_WARN_ONCE(
			"Checking for direct navigability between nodes is not yet implemented. This message will only print once.");

	//For now, it only check whether a node is within a certain distance...

	if (calcDistance(node1, node2) <= 5) //if (calcDistance(node1, node2) <= 1 * 1.2)
		return true;
	else
		return false;
	//@TODO: 1m limit is hardcoded again -> change it to become linked with node creation condition
	//*1.2 -> to give it some extra play, as nodes are created as soon as dist>1.
}

/*!
 * edgeExists
 */
const bool TopoNavMap::edgeExists(const TopoNavNode &node1,
		const TopoNavNode &node2) const {
	//TODO: if giving the edges and ID like the string "2to1", you will have unique IDs that are descriptive enough to facilitate edgeExists etc.
	ROS_WARN_ONCE(
			"edgeExists is not yet implemented. It should help block recreation of edges in checkCreateEdge. This goes well for new edges (there is no risk of duplicates), but triggering checkCreateEdge when updating a node for example will likely lead to duplicate edges. This message will only print once.");
	return false;
}

/*!
 * checkIsDoor
 */
bool TopoNavMap::checkIsNewDoor() {
//@TODO write this method
	ROS_WARN_ONCE(
			"Detecting/creating Doors is not yet implemented. This message will only print once.");
	return false;
}

/*!
 * distanceToClosestNode
 */
double TopoNavMap::distanceToClosestNode() {
// @TODO: This method compares to all nodes -> scales poorly eventually!
// One idea to make it scale slightly better:bool anyNodeCloserThen(max_dist), which return false if there isnt any (full search space needs to be searched) or returns true if there is (usually only first part of search space needs to be searched, if you start at end of nodes_ std::map)
	double dist, minimum_dist;
	int closest_node_id;
	int number_of_nodes = getNumberOfNodes();
	if (number_of_nodes == 0)
		minimum_dist = INFINITY; //No nodes means -> dist in inf.
	else {
		for (std::map<NodeID, TopoNavNode*>::iterator it=nodes_.begin(); it!=nodes_.end(); it++) {
			dist = calcDistance(*(it->second), robot_pose_tf_);

			ROS_DEBUG("Distance between Robot and Node_ID %d = %f",
					it->second->getNodeID(), dist);

			if (it == nodes_.begin() || dist < minimum_dist) {
				minimum_dist = dist;
				closest_node_id = it->second->getNodeID();
			}
		}
	}
	ROS_DEBUG("Minimum distance = [%f], Closest Node ID= [%d]", minimum_dist,
			closest_node_id);

	return minimum_dist;

}

/*!
 * addEdge
 */
void TopoNavMap::addEdge(const TopoNavNode &start_node,
		const TopoNavNode &end_node) {
	new TopoNavEdge(start_node, end_node, edges_); //Using "new", the object will not be destructed after leaving this method!
}

/*!
 * addNode
 */
void TopoNavMap::addNode(const tf::Pose &pose, bool is_door, int area_id) {
	new TopoNavNode(pose, is_door, area_id, nodes_); //Using "new", the object will not be destructed after leaving this method!
}

/*!
 * deleteEdge
 */
void TopoNavMap::deleteEdge(EdgeID edge_id) {
	deleteEdge(*edges_[edge_id]);
}
void TopoNavMap::deleteEdge(TopoNavEdge &edge) {
	delete &edge;
}

/*!
 * deleteNode
 */
void TopoNavMap::deleteNode(NodeID node_id) {
	deleteNode(*nodes_[node_id]);
}
void TopoNavMap::deleteNode(TopoNavNode &node) {
	std::map<EdgeID, TopoNavEdge*> connected_edges = connectedEdges(node);
	for (std::map<EdgeID, TopoNavEdge*>::iterator it=connected_edges.begin(); it!=connected_edges.end(); it++) {
		deleteEdge((*it->second));
	}
	delete &node;
}

std::map<EdgeID, TopoNavEdge*> TopoNavMap::connectedEdges(
		const TopoNavNode &node) const { //TODO scales poorly: all edges are checked!
	std::map<EdgeID, TopoNavEdge*> connected_edges;
	for (std::map<EdgeID, TopoNavEdge*>::const_iterator it=edges_.begin(); it!=edges_.end(); it++) {
		if (it->second->getStartNode().getNodeID() == node.getNodeID()
				|| it->second->getEndNode().getNodeID() == node.getNodeID()) {
			connected_edges[it->second->getEdgeID()]=(it->second);
		}
	}
	return connected_edges;
}


void TopoNavMap::nodeFromRosMsg(const st_topological_mapping::TopoNavNodeMsg node_msg, std::map<NodeID, TopoNavNode*> &nodes) {
	tf::Pose tfpose;
	poseMsgToTF(node_msg.pose,tfpose);

	new TopoNavNode(
			node_msg.node_id, //node_id
			node_msg.last_updated,//last_updated
			tfpose,//pose
			node_msg.is_door,//is_door
			node_msg.area_id,//area_id
			nodes//nodes map
	);
}

void TopoNavMap::edgeFromRosMsg(const st_topological_mapping::TopoNavEdgeMsg edge_msg, std::map<EdgeID, TopoNavEdge*> &edges) {
	new TopoNavEdge(edge_msg.edge_id, //edge_id
			edge_msg.last_updated, //last_updated
			edge_msg.cost, //cost
			*nodes_[edge_msg.start_node_id], //start_node
			*nodes_[edge_msg.end_node_id], //end_node
			edges //edges std::map
			);
}
st_topological_mapping::TopoNavEdgeMsg TopoNavMap::edgeToRosMsg(const TopoNavEdge* edge){
	st_topological_mapping::TopoNavEdgeMsg msg_edge;
	msg_edge.edge_id = edge->getEdgeID();
	msg_edge.last_updated = edge->getLastUpdatedTime();
	msg_edge.start_node_id = edge->getStartNode().getNodeID();
	msg_edge.end_node_id = edge->getEndNode().getNodeID();
	msg_edge.cost = edge->getCost();

	return msg_edge;
}

st_topological_mapping::TopoNavNodeMsg TopoNavMap::nodeToRosMsg(
		const TopoNavNode* node) {
	st_topological_mapping::TopoNavNodeMsg msg_node;
	msg_node.node_id = node->getNodeID();
	msg_node.last_updated = node->getLastUpdatedTime();
	msg_node.area_id = node->getAreaID();
	poseTFToMsg(node->getPose(), msg_node.pose);
	msg_node.is_door = node->getIsDoor();

	return msg_node;
}
