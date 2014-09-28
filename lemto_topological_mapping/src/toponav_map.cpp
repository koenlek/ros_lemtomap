/**
 * @file toponav_map
 * @brief Build and maintain the topological navigation map
 * @author Koen Lekkerkerker
 */

#include <lemto_topological_mapping/toponav_map.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i)) //to convert a map matrix coordinate to the index value in a map msg

//TODO - p2 - In general, the system should overall switch from normal poses, transforms, etc. to stamped poses, transforms, etc. Transforms are now manually programmed properly, but using stamped it is easier to keep track of in what frame a pose should be considered and some ros functions can automatically cast it to the right frames (e.g. move_base_goals are automatically interpreted in the required frame, you can pass it in any...).

/*!
 * \brief Constructor.
 */
TopoNavMap::TopoNavMap(ros::NodeHandle &n) :
    n_(n),
    max_edge_length_(2.5),
    new_node_distance_(1.0),
    associated_node_(-1),
    lalemto_bgl_affecting_update_(ros::WallTime::now())
{
  ros::NodeHandle private_nh("~");
  std::string scan_topic;

  // Parameters initialization
  private_nh.param("max_edge_creation", max_edge_creation_, bool(true));
  private_nh.param("scan_topic", scan_topic, std::string("scan"));
  private_nh.param("local_costmap_topic", local_costmap_topic_, std::string("move_base/local_costmap/costmap"));
  private_nh.param("global_costmap_topic", global_costmap_topic_, std::string("move_base/global_costmap/costmap"));
  private_nh.param("loop_closure_max_topo_dist", loop_closure_max_topo_dilemto_, double(100));
  private_nh.param("odom_frame", odom_frame_, std::string("odom"));

  // Set initial transform between map and toponav_map
  tf_toponavmap2map_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tf_toponavmap2map_.setRotation(q);
  br_.sendTransform(tf::StampedTransform(tf_toponavmap2map_, ros::Time::now(), tf_listener_.resolve("map"), tf_listener_.resolve("toponav_map")));

  //Create subscribers/publishers
  local_costmap_sub_ = n_.subscribe(local_costmap_topic_, 1, &TopoNavMap::lcostmapCB, this);
  if (max_edge_creation_) { //only subscribe if needed (to decrease load)
    global_costmap_sub_ = n_.subscribe(global_costmap_topic_, 1, &TopoNavMap::gcostmapCB, this);
  }

  toponav_map_pub_ = private_nh.advertise<lemto_topological_mapping::TopologicalNavigationMap>("topological_navigation_map", 1, true);
  asso_node_servserv_ = private_nh.advertiseService("get_associated_node", &TopoNavMap::associatedNodeSrvCB, this);
  predecessor_map_servserv_ = private_nh.advertiseService("get_predecessor_map", &TopoNavMap::predecessorMapSrvCB, this);
  directnav_servserv_ = private_nh.advertiseService("is_direct_navigable", &TopoNavMap::isDirectNavigableSrvCB, this);

  //update the map one time, at construction. This will create the first map node.
  tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10));
  tf_listener_.waitForTransform("map", odom_frame_, ros::Time(0), ros::Duration(10));
  updateMap();
  ROS_INFO("TopoNavMap object is initialized");

}

TopoNavMap::~TopoNavMap() {
  while (edges_.size() > 0)
  {
    delete edges_.rbegin()->second;
  }
  while (nodes_.size() > 0)
  {
    delete nodes_.rbegin()->second;
  }
  std::cerr << "~TopoNavMap: Deleting object -> all TopoNavNodes and TopoNavEdges are destructed"<< std::endl;
}

/*!
 * \brief Laser Callback. Update laser_scan_
 */

/*!
 * \brief Get Associated Node Service, such that other ROS nodes can know what is the associated node.
 */
bool TopoNavMap::associatedNodeSrvCB(lemto_topological_mapping::GetAssociatedNode::Request &req,
                                     lemto_topological_mapping::GetAssociatedNode::Response &res) {
  if (associated_node_ > 0)
      {
    res.asso_node_id = associated_node_;
    return true;
  }
  else
    return false;
}

/*!
 * \brief Get Predecessor Map Service, such that other ROS nodes can know what is the predessor map of a node.
 */
bool TopoNavMap::predecessorMapSrvCB(lemto_topological_mapping::GetPredecessorMap::Request &req,
                                     lemto_topological_mapping::GetPredecessorMap::Response &res) {
  int source_node_id = req.source_node_id;
  updateNodeBGLDetails(source_node_id);

  const TopoNavNode::PredecessorMapNodeID& predecessor_map = nodes_[source_node_id]->getPredecessorMap();

  for (TopoNavNode::PredecessorMapNodeID::const_iterator it = predecessor_map.begin(); it != predecessor_map.end(); it++) {
    res.nodes.push_back(it->first);
    res.predecessors.push_back(it->second);
  }

  return true;
}

/*!
 * \brief Check if something Is Direct Navigable...
 */
bool TopoNavMap::isDirectNavigableSrvCB(lemto_topological_mapping::IsDirectNavigable::Request &req,
                         lemto_topological_mapping::IsDirectNavigable::Response &res){
  // if bool global in request is not defined, it is perceived as false...

  tf::Point start_point, end_point;
  pointMsgToTF(req.start_point,start_point);
  pointMsgToTF(req.end_point,end_point);

  if (req.max_allowable_cost != 0)
    res.direct_navigable = directNavigable(start_point, end_point, req.global, req.max_allowable_cost);
  else
    res.direct_navigable = directNavigable(start_point, end_point, req.global);

  return true;
}

/*!
 * \brief Find the node id that is currently associated with the robot. I.e. the node that the robot is currently at.
 */
void TopoNavMap::updateAssociatedNode() {
  updateNodeBGLDetails(associated_node_);
  int associated_node_new = associated_node_;
  TopoNavNode::AdjacentNodes candidate_nodes = nodes_[associated_node_]->getAdjacentNodeIDs();
  candidate_nodes.push_back(associated_node_); //all adjacent nodes and current node
  double dilemto_metric_tmp;
  double dilemto_metric_min = DBL_MAX;
  for (int i = 0; i < candidate_nodes.size(); i++) {
    dilemto_metric_tmp = calcDistance(robot_pose_tf_, nodes_[candidate_nodes.at(i)]->getPoseInMap(tf_toponavmap2map_));
    if (dilemto_metric_tmp < dilemto_metric_min) {
      //ROS_INFO("Node %d is %.4f m away from the robot",candidate_nodes.at(i),dilemto_metric_min);
      dilemto_metric_min = dilemto_metric_tmp;
      associated_node_new = candidate_nodes.at(i);
    }
  }
  //ROS_INFO("Closest Node is %d, which is %.4f m away from the robot",associated_node_new,dilemto_metric_min);
  associated_node_ = associated_node_new;
}

/*!
 * \brief ToponavMapTransform()
 */
void TopoNavMap::updateToponavMapTransform() {

  tf::StampedTransform tf_toponavmap2map_stamped;

  try
  {
    tf_listener_.waitForTransform("map", odom_frame_, ros::Time(0), ros::Duration(2));
    tf_listener_.lookupTransform("map", odom_frame_, ros::Time(0), tf_toponavmap2map_stamped);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Error looking up transformation\n%s", ex.what());
  }
  tf_toponavmap2map_ = tf_toponavmap2map_stamped;

  br_.sendTransform(tf::StampedTransform(tf_toponavmap2map_, ros::Time::now(), "map", "toponav_map"));
}

/*!
 * \brief Local Costmap Callback.
 */
void TopoNavMap::lcostmapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_DEBUG("Local Costmap Callback");
  local_costmap_ = *msg;
  poseMsgToTF(local_costmap_.info.origin, local_costmap_origin_tf_);
  br_.sendTransform(tf::StampedTransform(local_costmap_origin_tf_, ros::Time::now(), local_costmap_.header.frame_id, "local_costmap_origin"));
}

/*!
 * \brief Global Costmap Callback.
 */
void TopoNavMap::gcostmapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_DEBUG("Global Costmap Callback");
  global_costmap_ = *msg;
  poseMsgToTF(global_costmap_.info.origin, global_costmap_origin_tf_);
  br_.sendTransform(tf::StampedTransform(global_costmap_origin_tf_, ros::Time::now(), global_costmap_.header.frame_id, "global_costmap_origin"));
}

/*!
 * \brief Publish the Topological Navigation Map.
 */
void TopoNavMap::publishTopoNavMap() {
  ROS_DEBUG("publishTopoNavMap");
  lemto_topological_mapping::TopologicalNavigationMap msg_map;

  msg_map.header.stamp = ros::Time::now();
  msg_map.header.frame_id = "toponav_map";

  for (TopoNavNode::NodeMap::iterator it = nodes_.begin(); it != nodes_.end(); it++)
      {
    msg_map.nodes.push_back(nodeToRosMsg(it->second));
  }
  for (TopoNavEdge::EdgeMap::iterator it = edges_.begin(); it != edges_.end(); it++)
      {
    msg_map.edges.push_back(edgeToRosMsg(it->second));
  }

  toponav_map_pub_.publish(msg_map);
}

/*!
 * \brief updateRobotPose
 */
void TopoNavMap::updateRobotPose() {
  try
  {
    /*
     * The /map frame is used to check the pose of the robot. The name /map is a bit confusing here.
     * /map can be regarded an improved version of /odom: the origin of /odom is the place where the
     * robot started according to the robots odometry, the origin of /map is the place where the
     * (rolling window) gmapping thinks the robot originally started, hence it can be regarded an
     * improved version of /odom. We kept the name /map, as it is the common frame to use for gmapping
     * and many other packages, and it is sometimes hardcoded. However, /odom_improved or something
     * like that would possibly make more sense.
     */
    tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2));
    tf_listener_.lookupTransform("map", "base_link", ros::Time(0), robot_transform_tf_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Error looking up transformation\n%s", ex.what());
  }

  robot_pose_tf_.setOrigin(robot_transform_tf_.getOrigin());
  robot_pose_tf_.setRotation(robot_transform_tf_.getRotation());

  ROS_DEBUG("Pose is x=%f, y=%f, theta=%f", robot_pose_tf_.getOrigin().x(),
            robot_pose_tf_.getOrigin().y(),
            tf::getYaw(robot_pose_tf_.getRotation()));
}

/*!
 * \brief loadSavedMap
 */
void TopoNavMap::loadSavedMap(const lemto_topological_mapping::TopologicalNavigationMap &toponavmap_msg, const int& associated_node, const geometry_msgs::Pose& robot_pose) {
  // todo - p2 - time shift in simulation should be fixed, asso node and pose loading can be added

  ROS_WARN("Loading Saved Map: Loading saved associated node and robot pose is not yet supported, will default to node 1 and (x,y,theta) = (0,0,0)");

  /*
  // set associated node
  associated_node_ = associated_node;

  // set robot pose compared to topological graph
  // this is not yet implemented. This should make sure that the robot has the right position compared to the nodes of the graph...

  // set robot pose in Gazebo (should be only used if it is a simulated experiment)
  ros::ServiceClient gazebo_setmodelstate_servcli =  n_.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");

  gazebo_msgs::SetModelState srv;
  srv.request.model_state.model_name = "mobile_base";
  srv.request.model_state.pose.position = robot_pose.position;
  srv.request.model_state.pose.orientation = robot_pose.orientation;

  if (gazebo_setmodelstate_servcli.call(srv)) {
    ROS_DEBUG("Robot pose is updated through /gazebo/set_model_state service ");
  }
  else {
    ROS_ERROR("Failed to call service /gazebo/set_model_state");
  }*/

  // load nodes and edges from toponavmap_msg
  nodes_.clear();
  edges_.clear();

  ROS_WARN("Loading Saved Map: When loading topo map from msg for a simulation, node and edge update times should be shifted such that they are all before the current ros::Time::now() (OR ros::Time::now() should be shifted backwards). NOTE: in the current implementation, this should not yet cause issues, as the update times aren't used yet. BGL update times are all initialized as 0, as they all need to be updated (topo map msg does not containt BGL info)");

  for (int i = 0; i < toponavmap_msg.nodes.size(); i++)
      {
    nodeFromRosMsg(toponavmap_msg.nodes.at(i));
    ROS_DEBUG("Loaded node with ID=%lu to std::map nodes_", toponavmap_msg.nodes.at(i).node_id);
  }
  for (int i = 0; i < toponavmap_msg.edges.size(); i++)
      {
    edgeFromRosMsg(toponavmap_msg.edges.at(i));
    ROS_DEBUG("Loaded edge with ID=%lu to std::map edges_", toponavmap_msg.nodes.at(i).node_id);
  }
  ROS_INFO("Finished loading the TopoNavMap");
}

/*!
 * \brief updateMap
 */
void TopoNavMap::updateMap() {

  updateRobotPose();

  updateToponavMapTransform();

  if (!checkCreateNode()) { //creates Nodes and Edges
    updateAssociatedNode(); //updates are only needed if no new node was created...
  }

  publishTopoNavMap();

}

/*!
 * \brief checkCreateNewNode
 */
bool TopoNavMap::checkCreateNode() {
  int number_of_nodes = getNumberOfNodes();
  int area_id = 1; //FIXME - p2 - area_id is always 1!
  bool create_node = false;
  bool is_door = false;

  if (number_of_nodes == 0) { // if no nodes, create first and return
    addNode(robot_pose_tf_, is_door, area_id);
    associated_node_ = nodes_.rbegin()->second->getNodeID(); //update associated node...
    updateNodeBGLDetails(nodes_.rbegin()->second->getNodeID());
    return true;
  }

  double asso_node_dist = calcDistance(robot_pose_tf_, nodes_[associated_node_]->getPoseInMap(tf_toponavmap2map_));

  if (checkIsNewDoor()) {
    //TODO - p3 - later, maybe door nodes should not influence other nodes. Maybe they should not be regular nodes at all. Check SAS10 for comparison.
    create_node = true;
    is_door = true;
  }
  else if (asso_node_dist > new_node_distance_) {
    create_node = true;
  }
  if (create_node) {
    addNode(robot_pose_tf_, is_door, area_id);
    checkCreateEdges(); // try to create additional edges!
    associated_node_ = nodes_.rbegin()->second->getNodeID(); //update associated node...
    updateNodeBGLDetails(associated_node_); //this is maybe not necessary?
    return true;
  }
  else {
    ROS_DEBUG("No new node created");
    return false;
  }
}

/*!
 * \brief checkCreateEdges: should only be used for new nodes where the robot is currently at (as it needs a sufficient large costmap around the node)
 */
void TopoNavMap::checkCreateEdges() {
  TopoNavNode &node = (*nodes_.rbegin()->second);
  if (getNumberOfNodes() < 2)
    return; //only continue if there are 2 or more nodes

  addEdge(node, *nodes_[associated_node_], 1);  //create at least edge between the new and (previous) associated_node one!

  updateNodeBGLDetails(node.getNodeID()); // this is needed as distance map is needed to check for loop_closure_max_topo_dilemto_
  TopoNavNode::DistanceBiMapNodeID dilemto_map = node.getDistanceMap();

  if (max_edge_creation_) {
    for (TopoNavNode::DistanceBiMapNodeID::right_map::const_iterator right_iter = dilemto_map.right.begin(); right_iter != dilemto_map.right.end(); right_iter++) {
      if (right_iter->first < loop_closure_max_topo_dilemto_) {
        if (right_iter->second == node.getNodeID()) //not compare to self!
          continue;
        else if (!isInCostmap(right_iter->second, true)) //dont check if the node is outside of the area covered by de occupancy grid map
          continue;
        else if (!edgeExists(node.getNodeID(), right_iter->second)) { //not check if already exists (otherwise edge to prev. asso. node will be double created...)
          if (directNavigable(node.getPoseInMap(tf_toponavmap2map_).getOrigin(), nodes_[right_iter->second]->getPoseInMap(tf_toponavmap2map_).getOrigin(), true)) //only create if directNavigable.
            if (calcDistance(node, *nodes_[right_iter->second]) < 4.0) //todo - p1 - This is unforatunately necessary as a solution to a limitation of the global planner. The max 4.5m limit is added here to make sure next nodes in a topological planning path cannot be outside of the sliding window (i.e. outside of the global costmap). 5m is used as the border of the window shifts if the laser scans get outside of the window, so with a max range of 5.6m for the scanner, 4.0 should be safe.
              addEdge(node, *nodes_[right_iter->second], 2);
        }
        //ROS_INFO("NodeID %d, has dist %.4f", right_iter->second, right_iter->first);
      }
      else
        break; //as the right version of the bimap is ordered by topo distance, we can break as soon as we have pass max_topo_dist
    }
  }
  else {
    for (TopoNavNode::DistanceBiMapNodeID::right_map::const_iterator right_iter = dilemto_map.right.begin(); right_iter != dilemto_map.right.end(); right_iter++) {
      if (right_iter->first < loop_closure_max_topo_dilemto_) {
        if (right_iter->second == node.getNodeID()) //not compare to self!
          continue;
        else if (calcDistance(node, *nodes_[right_iter->second]) > max_edge_length_) //not check if > max_edge_length_
          continue;
        else if (!edgeExists(node.getNodeID(), right_iter->second)) { //not check if already exists
          if (directNavigable(node.getPoseInMap(tf_toponavmap2map_).getOrigin(), nodes_[right_iter->second]->getPoseInMap(tf_toponavmap2map_).getOrigin(), false)) //only create if directNavigable.
            addEdge(node, *nodes_[right_iter->second], 2);
        }
        //ROS_INFO("NodeID %d, has dist %.4f", right_iter->second, right_iter->first);
      }
      else
        break; //as the right version of the bimap is ordered by topo distance, we can break as soon as we have pass max_topo_dist
    }
  }
  return;
}

/*!
 * \brief directNavigable
 */
const bool TopoNavMap::directNavigable(const tf::Point &point1, const tf::Point &point2, bool global, int max_allowable_cost) {
  bool navigable = false;

  int line_cost;
  line_cost = getCMLineCost(point1, point2, global);
  ROS_DEBUG("Straight line in toponav_map from (x1,y1)=(%.4f,%.4f) to (x2,y2)=(%.4f,%.4f) has cost: %d",
            point1.getX(),
            point1.getY(),
            point2.getX(),
            point2.getY(),
            line_cost);
  // 100 is LETHAL (obstacle), 99 is INSCRIBED (will hit due to robot footprint), -1 is unknown.
  // See: http://wiki.ros.org/costmap_2d
  // See the colemto_translation_table_ here: https://github.com/ros-planning/navigation/blob/hydro-devel/costmap_2d/src/costmap_2d_publisher.cpp
  // max_allowable_cost is set to 90 by default in the prototype definition
  if (line_cost <= max_allowable_cost && line_cost >= 0)
    navigable = true;

  //ROS_INFO ("Used max allowable cost of %d",max_allowable_cost);

  return navigable;
}

/**\brief turn a /map tf::Point into a cell of the local or global costmap
 * \param map_coordinate The coordiante in the /map
 * \param cell_x (output) The gridcell coordinate x
 * \param cell_y (output) The gridcell coordinate y
 * \return N/A
 */
bool TopoNavMap::mapPoint2costmapCell(const tf::Point &map_coordinate, int &cell_i, int &cell_j, bool global) const {
  // create aliases
  const nav_msgs::OccupancyGrid *costmap;
  std::string costmap_origin;
  if (global) {
    costmap = &global_costmap_;
    costmap_origin = "global_costmap_origin";
  }
  else {
    costmap = &local_costmap_;
    costmap_origin = "local_costmap_origin";
  }

  bool validpoint = true;
  std::string costmap_frame_id = costmap->header.frame_id; //global frame as specified in move_base local/global costmap params

  tf::Stamped<tf::Point> map_coordinate_stamped(map_coordinate, ros::Time(0), "map");
  tf::Stamped<tf::Point> map_coordinate_incostmap_origin;
  try
  {
    tf_listener_.waitForTransform(costmap_origin, "map", ros::Time(0), ros::Duration(2.0)); //not necessary
    tf_listener_.transformPoint(costmap_origin, map_coordinate_stamped, map_coordinate_incostmap_origin);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  ROS_DEBUG("map_coordinate_incostmap_origin x=%.4f , y=%.4f", map_coordinate_incostmap_origin.getX(), map_coordinate_incostmap_origin.getY());

  cell_i = floor(map_coordinate_incostmap_origin.getY() / costmap->info.resolution); // i is row, i.e. y
  cell_j = floor(map_coordinate_incostmap_origin.getX() / costmap->info.resolution); // j is column, i.e. x

  if (cell_i < 0 || cell_j < 0 || cell_i >= costmap->info.height || cell_j >= costmap->info.width)
      {
    ROS_ERROR("mapPoint2costmapCell: Index out of bounds!");
    validpoint = false;
  }
  return validpoint;
}

/**\brief Find the max. cost between two points
 * \param point1 start point as a coordinate in /map
 * \param point2 end point as a coordinate in /map
 * \return cost - 0 is no obstacles, 100 is blocking obstacle. Anything in between is a degree of blocking...
 */
int TopoNavMap::getCMLineCost(const tf::Point &point1, const tf::Point &point2, bool global) const {
  int cell1_i, cell1_j, cell2_i, cell2_j;
  if (mapPoint2costmapCell(point1, cell1_i, cell1_j, global) && mapPoint2costmapCell(point2, cell2_i, cell2_j, global))
    return getCMLineCost(cell1_i, cell1_j, cell2_i, cell2_j, global);
  else
    return INT_MAX;
}

/** \brief getCMLineCost: Find the max. cost between two points */
int TopoNavMap::getCMLineCost(const int &cell1_i, const int &cell1_j, const int &cell2_i, const int &cell2_j, bool global) const {
  // create aliases
  const nav_msgs::OccupancyGrid *costmap;
  if (global) {
    costmap = &global_costmap_;
  }
  else {
    costmap = &local_costmap_;
  }
  // This code was largely based on the CostmapModel::lineCost function
  // Defined here: http://docs.ros.org/hydro/api/base_local_planner/html/costmap__model_8cpp_source.html
  int line_cost = 0.0;
  int point_cost = -1.0;

  for (base_local_planner::LineIterator line(cell1_i, cell1_j, cell2_i, cell2_j); line.isValid(); line.advance()) {
    //point_cost = (*costmap_matrix)(line.getX(), line.getY()); //Score the current point
    point_cost = costmap->data[MAP_IDX(costmap->info.width, line.getY(), line.getX())];

    if (point_cost < 0)
      return -1;

    if (line_cost < point_cost)
      line_cost = point_cost;
  }
  return line_cost;
}

/*!
 * \brief isInMap
 */
bool TopoNavMap::isInCostmap(TopoNavNode::NodeID nodeid, bool global) {
  double x, y;
  x = nodes_[nodeid]->getPoseInMap(tf_toponavmap2map_).getOrigin().getX();
  y = nodes_[nodeid]->getPoseInMap(tf_toponavmap2map_).getOrigin().getY();
  return isInCostmap(x, y, global);
}
bool TopoNavMap::isInCostmap(double x, double y, bool global) {
  // create aliases
  const nav_msgs::OccupancyGrid *costmap;
  if (global) {
    costmap = &global_costmap_;
  }
  else {
    costmap = &local_costmap_;
  }
  // check if it is inside
  double xmin, xmax, ymin, ymax;
  bool is_inside = false;
  xmin = costmap->info.origin.position.x;
  ymin = costmap->info.origin.position.y;
  xmax = costmap->info.origin.position.x + costmap->info.width * costmap->info.resolution;
  ymax = costmap->info.origin.position.y + costmap->info.height * costmap->info.resolution;

  if (x < xmax && x > xmin && y < ymax && y > ymin)
    is_inside = true;
  return is_inside;
}

/*!
 * \brief edgeExists
 */
const bool TopoNavMap::edgeExists(const TopoNavNode::NodeID &nodeid1, const TopoNavNode::NodeID &nodeid2) const {
  std::string edge_id;
  if (nodeid1 < nodeid2) {
    edge_id = boost::lexical_cast<std::string>(nodeid1) + "to" + boost::lexical_cast<std::string>(nodeid2);
  }
  else {
    edge_id = boost::lexical_cast<std::string>(nodeid2) + "to" + boost::lexical_cast<std::string>(nodeid1);
  }
  return edges_.find(edge_id) != edges_.end();
}

/*!
 * \brief checkIsNewDoor
 */
bool TopoNavMap::checkIsNewDoor() {
// TODO - p3 - write this method
  ROS_WARN_ONCE(
                "Detecting/creating Doors is not yet implemented. This message will only print once.");
  return false;
}

/*!
 * \brief distanceToClosestNode
 */
double TopoNavMap::distanceToClosestNode() {
// TODO - p3 - This method compares to all nodes -> scales poorly eventually!
// One idea to make it scale slightly better:bool anyNodeCloserThen(max_dist), which return false if there isnt any (full search space needs to be searched) or returns true if there is (usually only first part of search space needs to be searched, if you start at end of nodes_ std::map)
  double dist, minimum_dist;
  int closelemto_node_id;
  int number_of_nodes = getNumberOfNodes();
  if (number_of_nodes == 0)
    minimum_dist = INFINITY; //No nodes means -> dist in inf.
  else
  {
    for (TopoNavNode::NodeMap::iterator it = nodes_.begin(); it != nodes_.end(); it++)
        {
      dist = calcDistance(it->second->getPoseInMap(tf_toponavmap2map_), robot_pose_tf_);

      ROS_DEBUG("Distance between Robot and Node_ID %d = %f",
                it->second->getNodeID(),
                dist);

      if (it == nodes_.begin() || dist < minimum_dist)
          {
        minimum_dist = dist;
        closelemto_node_id = it->second->getNodeID();
      }
    }
  }
  ROS_DEBUG("Minimum distance = [%f], Closest Node ID= [%d]", minimum_dist,
            closelemto_node_id);

  return minimum_dist;

}

/*!
 * \brief addEdge
 */
void TopoNavMap::addEdge(const TopoNavNode &start_node,
                         const TopoNavNode &end_node, int type) {
  new TopoNavEdge(start_node, end_node, type, edges_, lalemto_bgl_affecting_update_); //Using "new", the object will not be destructed after leaving this method!
}

/*!
 * \brief addNode
 */
void TopoNavMap::addNode(const tf::Pose &pose, bool is_door, int area_id) {
  new TopoNavNode(tf_toponavmap2map_.inverse() * pose, is_door, area_id, nodes_, lalemto_bgl_affecting_update_); //Using "new", the object will not be destructed after leaving this method!
}

/*!
 * \brief deleteEdge
 */
void TopoNavMap::deleteEdge(TopoNavEdge::EdgeID edge_id) {
  deleteEdge(*edges_[edge_id]);

}
void TopoNavMap::deleteEdge(TopoNavEdge &edge) {
  delete &edge;
}

/*!
 * \brief deleteNode
 */
void TopoNavMap::deleteNode(TopoNavNode::NodeID node_id) {
  deleteNode(*nodes_[node_id]);
}

void TopoNavMap::deleteNode(TopoNavNode &node) {
  updateNodeBGLDetails(node.getNodeID()); //to make sure connected_edges are up to date!
  TopoNavNode::AdjacentEdges connected_edges = node.getAdjacentEdgeIDs();
  for (int i = 0; i < connected_edges.size(); i++)
      {
    deleteEdge((*edges_[connected_edges.at(i)]));
  }
  delete &node;
}

void TopoNavMap::updateNodeBGLDetails(TopoNavNode::NodeID node_id) {
  lemto_bgl::updateNodeDetails(nodes_, edges_, node_id, lalemto_bgl_affecting_update_);
}

void TopoNavMap::nodeFromRosMsg(const lemto_topological_mapping::TopoNavNodeMsg &node_msg) {
  tf::Pose tfpose;
  poseMsgToTF(node_msg.pose, tfpose);

  new TopoNavNode(node_msg.node_id, //node_id
  node_msg.lalemto_updated, //lalemto_updated
  node_msg.lalemto_pose_updated, //lalemto_pose_updated
  ros::WallTime(0), //lalemto_bgl_update set to 0, which will make any consulting trigger update!
  tfpose, //pose
  node_msg.is_door, //is_door
  node_msg.area_id, //area_id
  nodes_, //nodes map
  lalemto_bgl_affecting_update_ //lalemto_toponavmap_bgl_affecting_update
  );
}

void TopoNavMap::edgeFromRosMsg(const lemto_topological_mapping::TopoNavEdgeMsg &edge_msg) {
  new TopoNavEdge(edge_msg.edge_id, //edge_id
  edge_msg.lalemto_updated, //lalemto_updated
  edge_msg.cost, //cost
  *nodes_[edge_msg.start_node_id], //start_node
  *nodes_[edge_msg.end_node_id], //end_node
  edge_msg.type,
                  edges_, //edges std::map
                  lalemto_bgl_affecting_update_ //lalemto_toponavmap_bgl_affecting_update
                  );
}
lemto_topological_mapping::TopoNavEdgeMsg
TopoNavMap::edgeToRosMsg(TopoNavEdge *edge) { //not const, as getCost can cause update of the cost!
  lemto_topological_mapping::TopoNavEdgeMsg msg_edge;
  msg_edge.edge_id = edge->getEdgeID();
  msg_edge.lalemto_updated = edge->getLastUpdatedTime();
  msg_edge.start_node_id = edge->getStartNode().getNodeID();
  msg_edge.type = edge->getType();
  msg_edge.end_node_id = edge->getEndNode().getNodeID();
  msg_edge.cost = edge->getCost();

  return msg_edge;
}

lemto_topological_mapping::TopoNavNodeMsg TopoNavMap::nodeToRosMsg(const TopoNavNode *node) {
  lemto_topological_mapping::TopoNavNodeMsg msg_node;
  msg_node.node_id = node->getNodeID();
  msg_node.lalemto_updated = node->getLastUpdatedTime();
  msg_node.lalemto_pose_updated = node->getLastPoseUpdateTime();
  msg_node.area_id = node->getAreaID();
  poseTFToMsg(node->getPose(), msg_node.pose);
  msg_node.is_door = node->getIsDoor();

  return msg_node;
}
