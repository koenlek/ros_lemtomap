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
	while (nodes_.size() > 0) {
		delete nodes_.back();
	}

	while (edges_.size() > 0) {
		delete edges_.back();
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

	for (int i = 0; i < nodes_.size(); i++) {
		msg_map.nodes.push_back(nodeToRosMsg(nodes_.at(i)));
	}
	for (int i = 0; i < edges_.size(); i++) {
		msg_map.edges.push_back(edgeToRosMsg(edges_.at(i)));
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
		ROS_DEBUG("Loaded node with ID=%d to vector.at place %d", nodes_.at(i)->getNodeID(),i);
	}
	for (int i = 0; i < toponavmap_msg.edges.size(); i++) {
		new TopoNavEdge(
				toponavmap_msg.edges.at(i).edge_id, //edge_id
				toponavmap_msg.edges.at(i).last_updated, //last_updated
				toponavmap_msg.edges.at(i).cost, //cost
				getNodeByID(toponavmap_msg.edges.at(i).start_node_id), //start_node
				getNodeByID(toponavmap_msg.edges.at(i).end_node_id), //end_node
				edges_ //edges vector
				);
		ROS_DEBUG("Loaded edge with ID=%d to vector.at place %d", edges_.at(i)->getEdgeID(),i);
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

	 if (ros::Time().now()>ros::Time(35) && test_executed_==0)
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
	 tf::Pose tmp_pose=getNodeByID(4).getPose();
	 tmp_pose.getOrigin().setY(tmp_pose.getOrigin().getY()+0.3);
	 getNodeByID(4).setPose(tmp_pose);
	 test_executed_++;
	 }

#endif
}

/*!
 * getNodeByID
 */
TopoNavNode& TopoNavMap::getNodeByID(int node_id) {
	return *(nodes_.at(getNodeVectorPositionByID(node_id)));
}

/*!
 * getNodeVectorPosition
 */
const int TopoNavMap::getNodeVectorPosition(const TopoNavNode &node) const {
	return getNodeVectorPositionByID(node.getNodeID());
}

/*!
 * getNodeVectorPositionByID
 */
const int TopoNavMap::getNodeVectorPositionByID(node_id_int node_id) const {
	// @TODO: This method uses simple heuristics to make sure not too many nodes are searched. Maybe this can be done more clever?
	int node_vector_position = 0;
	int number_of_nodes = getNumberOfNodes();

	if (number_of_nodes == 0) {
		ROS_FATAL(
				"There are zero nodes in the nodes_ vector. Therefore, this ROS Node will now shutdown.");
		n_.shutdown();
		return 0;
	}
	while (node_id != nodes_.at(node_vector_position)->getNodeID()) {
		ROS_DEBUG(
				"Checked position [%d] in nodes_ for node_id [%d], which mismatched as it actually has node_id [%d]",
				node_vector_position, node_id,
				nodes_.at(node_vector_position)->getNodeID());
		if (node_vector_position + 1 == number_of_nodes)
			break; //break out of this while loop as soon as all vector positions are checked
		node_vector_position++;
	}
	if (node_id == nodes_.at(node_vector_position)->getNodeID()) {
		ROS_DEBUG(
				"Node found! Node_ID [%d] has position [%d] in the nodes_ vector",
				node_id, node_vector_position);
	} else {
		ROS_FATAL(
				"There is no node with Node ID %d in the nodes_ vector. Therefore, this ROS Node will now shutdown.",
				node_id);
		n_.shutdown();
	}
	return node_vector_position;
}

/*!
 * getEdgeByID
 */
TopoNavEdge& TopoNavMap::getEdgeByID(edge_id_int edge_id) {
	return *(edges_.at(getEdgeVectorPositionByID(edge_id)));
}

/*!
 * getEdgeVectorPosition
 */
const int TopoNavMap::getEdgeVectorPosition(const TopoNavEdge &edge) const {
	return getEdgeVectorPositionByID(edge.getEdgeID());
}

/*!
 * getEdgeVectorPositionByID
 */
const int TopoNavMap::getEdgeVectorPositionByID(edge_id_int edge_id) const {
	// @TODO: This method uses simple heuristics to make sure not too many edges are searched. Maybe this can be done more clever?
	int edge_vector_position = 0;
	int number_of_edges = getNumberOfEdges();

	if (number_of_edges == 0) {
		ROS_FATAL(
				"There are zero edges in the edges_ vector. Therefore, this ROS Node will now shutdown.");
		n_.shutdown();
		return 0;
	}

	while (edge_id != edges_.at(edge_vector_position)->getEdgeID()) {
		ROS_DEBUG(
				"Checked position [%d] in edges_ for edge_id [%d], which mismatched as it actually has edge_id [%d]",
				edge_vector_position, edge_id,
				edges_.at(edge_vector_position)->getEdgeID());
		if (edge_vector_position + 1 == number_of_edges)
			break; //break out of this while loop as soon as all vector positions are checked
		edge_vector_position++;
	}

	if (edge_id == edges_.at(edge_vector_position)->getEdgeID()) {
		ROS_DEBUG(
				"Edge found! Edge_ID [%d] has position [%d] in the edges_ vector",
				edge_id, edge_vector_position);
	} else {
		ROS_FATAL(
				"There is no edge with Edge ID %d in the edges_ vector. Therefore, this ROS Node will now shutdown.",
				edge_id);
		n_.shutdown();
	}
	return edge_vector_position;
}

/*!
 * checkCreateNewNode
 */
bool TopoNavMap::checkCreateNode() {
	int number_of_nodes = getNumberOfNodes();
	int area_id = 1; //@TODO FIXME: room_id is always 1!
	bool create_node = false;
	bool is_door = false;

	if (checkIsNewDoor()) { //I
//@TODO: later, maybe door nodes should not influence other nodes. Maybe they should not be regular nodes at all. Check SAS10 for comparison.
		create_node = true;
		is_door = true;
	} else if (distanceToClosestNode() > 1) {
//@TODO FIXME: Remove magic number "1", which is the min distance here...
		create_node = true;
	}
	if (create_node) {
		addNode(robot_pose_tf_, is_door, area_id);
		checkCreateEdges(*nodes_.back());

		/*ROS_INFO("nodes_.at(0) Area ID = %d",getNodes().at(0)->getAreaID()); // This code can be used to check r/w access to nodes_ and edges_ member through their get Methods (read only is desired!)
		 getNodes().at(0)->setAreaID(3);
		 ROS_INFO("After setAreaID(3), nodes_.at(0) Area ID = %d",nodes_.at(0)->getAreaID());*/

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
	int node_vecpos = getNodeVectorPosition(node);
	bool edge_created = false;
	if (getNumberOfNodes() >= 2) {
		for (int i = 0; i < getNumberOfNodes(); i++) {
			if (i == node_vecpos)
				continue; //Not compare with itself
			if (!edgeExists(node, *nodes_.at(i))
					&& directNavigable(node, *nodes_.at(i))) {
				addEdge(node, *nodes_.at(i));
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
// One idea to make it scale slightly better:bool anyNodeCloserThen(max_dist), which return false if there isnt any (full search space needs to be searched) or returns true if there is (usually only first part of search space needs to be searched, if you start at end of nodes_ vector)
	double dist, minimum_dist;
	int closest_node_id;
	int number_of_nodes = getNumberOfNodes();
	if (number_of_nodes == 0)
		minimum_dist = INFINITY; //No nodes means -> dist in inf.
	else {
		for (int i = 0; i < number_of_nodes; i++) {
			dist = calcDistance(*nodes_.at(i), robot_pose_tf_);

			ROS_DEBUG("Distance between Robot and Node_ID %d = %f",
					nodes_.at(i)->getNodeID(), dist);

			if (i == 0 || dist < minimum_dist) {
				minimum_dist = dist;
				closest_node_id = nodes_.at(i)->getNodeID();
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
void TopoNavMap::deleteEdge(edge_id_int edge_id) {
	deleteEdge(getEdgeByID(edge_id));
}
void TopoNavMap::deleteEdge(TopoNavEdge &edge) {
	delete &edge;
}

/*!
 * deleteNode
 */
void TopoNavMap::deleteNode(node_id_int node_id) {
	deleteNode(getNodeByID(node_id));
}
void TopoNavMap::deleteNode(TopoNavNode &node) {
	std::vector<TopoNavEdge*> connected_edges = connectedEdges(node);
	for (int i = 0; i < connected_edges.size(); i++) {
		deleteEdge(*connected_edges.at(i));
	}
	delete &node;
}

std::vector<TopoNavEdge*> TopoNavMap::connectedEdges(
		const TopoNavNode &node) const { //TODO scales poorly: all edges are checked!
	std::vector<TopoNavEdge*> connected_edges;
	for (int i = 0; i < edges_.size(); i++) {
		if (edges_.at(i)->getStartNode().getNodeID() == node.getNodeID()
				|| edges_.at(i)->getEndNode().getNodeID() == node.getNodeID()) {
			connected_edges.push_back(edges_.at(i));
		}
	}
	return connected_edges;
}


void TopoNavMap::nodeFromRosMsg(const st_topological_mapping::TopoNavNodeMsg node_msg, std::vector<TopoNavNode*> &nodes) {
	tf::Pose tfpose;
	poseMsgToTF(node_msg.pose,tfpose);

	new TopoNavNode(
			node_msg.node_id, //node_id
			node_msg.last_updated,//last_updated
			tfpose,//pose
			node_msg.is_door,//is_door
			node_msg.area_id,//area_id
			nodes//nodes vector
	);
}

void TopoNavMap::edgeFromRosMsg(
		const st_topological_mapping::TopoNavEdgeMsg edge_msg,
		std::vector<TopoNavEdge*> &edges) {
	new TopoNavEdge(edge_msg.edge_id, //edge_id
			edge_msg.last_updated, //last_updated
			edge_msg.cost, //cost
			getNodeByID(edge_msg.start_node_id), //start_node
			getNodeByID(edge_msg.end_node_id), //end_node
			edges //nodes vector
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
