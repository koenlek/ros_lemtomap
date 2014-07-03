/**
 * @file toponav_map
 * @brief Build and maintain the topological navigation map
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/toponav_map.h>

//TODO - p2 - In general, the system should overall switch from normal poses, transforms, etc. to stamped poses, transforms, etc. Transforms are now manually programmed properly, but using stamped it is easier to keep track of in what frame a pose should be considered and some ros functions can automatically cast it to the right frames (e.g. move_base_goals are automatically interpreted in the required frame, you can pass it in any...).

/*!
 * \brief Constructor.
 */
TopoNavMap::TopoNavMap(ros::NodeHandle &n) :
    n_(n),
    costmap_lastupdate_seq_(0),
    max_edge_length_(2.5),
    new_node_distance_(1.0),
    move_base_client_("move_base", true), // this way, TopoNavMap is aware of the NodeHandle of this ROS node, just as ShowTopoNavMap will be...
    associated_node_(-1),
    last_bgl_affecting_update_(ros::WallTime::now())
{
  ros::NodeHandle private_nh("~");
  std::string scan_topic;
  std::string local_costmap_topic;

  // Parameters initialization
  private_nh.param("scan_topic", scan_topic, std::string("scan"));
  private_nh.param("local_costmap_topic", local_costmap_topic, std::string("move_base/local_costmap/costmap"));

  // Set initial transform between map and toponav_map
  tf_toponavmap2map_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tf_toponavmap2map_.setRotation(q);
  br_.sendTransform(tf::StampedTransform(tf_toponavmap2map_, ros::Time::now(), tf_listener_.resolve("map"), tf_listener_.resolve("toponav_map")));

  // Check for local costmap dimensions
  ROS_INFO("Waiting for move_base action server");
  move_base_client_.waitForServer();
  ROS_INFO("Waiting for move_base action server -- Finished");
  int cm_height, cm_width, cm_min;
  n_.param("move_base/local_costmap/height", cm_height, int(0));
  n_.param("move_base/local_costmap/width", cm_width, int(0));
  cm_min = std::min(cm_height, cm_width);
  if (max_edge_length_ > cm_min / 2 - 0.3) // when new edges are created, begin and end need to be in local costmap. The robot is *approx.* in the middle of the local costmap. - 0.3 is to give it some play for safety as it is only approx. in the middle.
    ROS_ERROR("The maximum distances between nodes was specified to be %.3fm, while the local costmap is %d x %dm. This is too small: cost lookups for edge creating have a high risk to become out of bounds", max_edge_length_, cm_height, cm_width);

  //Create subscribers/publishers
  fakeplan_client_ = n_.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan");
  local_costmap_sub_ = n_.subscribe(local_costmap_topic, 1, &TopoNavMap::lcostmapCB, this);
#if DEPRECATED
  scan_sub_ = n_.subscribe(scan_topic, 1, &TopoNavMap::laserCB, this);
#endif
  toponav_map_pub_ = private_nh.advertise<st_topological_mapping::TopologicalNavigationMap>("topological_navigation_map", 1, true);
  asso_node_servserv_ = n_.advertiseService("get_associated_node", &TopoNavMap::associatedNodeSrvCB, this);
  predecessor_map_servserv_ = n_.advertiseService("get_predecessor_map", &TopoNavMap::predecessorMapSrvCB, this);

  //update the map one time, at construction. This will create the first map node.
  updateMap();
  ROS_INFO("TopoNavMap object is initialized");

#if DEBUG
  test_executed_ = 0;
  counter_ = 0;
  last_run_update_max_ = 0;
  initialpose_sub_ = n_.subscribe("initialpose", 10, &TopoNavMap::initialposeCB, this);
#endif
}

TopoNavMap::~TopoNavMap()
{
  while (edges_.size() > 0)
  {
    delete edges_.rbegin()->second;
  }
  while (nodes_.size() > 0)
  {
    delete nodes_.rbegin()->second;
  }
  std::cerr
  << "~TopoNavMap: Deleting object -> all TopoNavNodes and TopoNavEdges are destructed"
      << std::endl;
}

/*!
 * \brief Laser Callback. Update laser_scan_
 */
#if DEPRECATED
void TopoNavMap::laserCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  laser_scan_ = *msg;
  ROS_DEBUG("angle_max=%f", laser_scan_.angle_max); // to check whether it uses kinect vs hokuyo
}
#endif

/*!
 * \brief Get Associated Node Service, such that other ROS nodes can know what is the associated node.
 */
bool TopoNavMap::associatedNodeSrvCB(st_topological_mapping::GetAssociatedNode::Request &req,
                                     st_topological_mapping::GetAssociatedNode::Response &res)
{
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
bool TopoNavMap::predecessorMapSrvCB(st_topological_mapping::GetPredecessorMap::Request &req,
                                     st_topological_mapping::GetPredecessorMap::Response &res)
{
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
 * \brief Find the node id that is currently associated with the robot. I.e. the node that the robot is currently at.
 */
void TopoNavMap::updateAssociatedNode()
{
  //updateAssociatedNode_method1();
  updateAssociatedNode_method2();

}

#if DEPRECATED
void TopoNavMap::updateAssociatedNode_method1()
{
  double closest_dist = DBL_MAX;
  double tentative_closest_dist;

  //FIXME - p1 - This loop picks the closest node, not even taking into account blocking obstacles or e.g. the direction in which the robot should travel eventually.
  ROS_WARN_ONCE("Currently (in this method: updateAssociatedNode_method1), the starting node is the closest node to robot, which could even go through walls. This method should be improved. This warning will only print once.");
  for (TopoNavNode::NodeMap::iterator it = nodes_.begin(); it != nodes_.end(); it++)
  {
    tentative_closest_dist = calcDistance(*(it->second), robot_pose_tf_);
    if (tentative_closest_dist < closest_dist)
    {
      closest_dist = tentative_closest_dist;
      associated_node_ = it->second->getNodeID();
    }
  }
  ROS_DEBUG("Associated Node ID=%d. Pose is x=%f, y=%f, theta=%f", associated_node_, robot_pose_tf_.getOrigin().x(), robot_pose_tf_.getOrigin().y(), tf::getYaw(robot_pose_tf_.getRotation()));
}
#endif

void TopoNavMap::updateAssociatedNode_method2()
{
  updateNodeBGLDetails(associated_node_);
  int associated_node_new = associated_node_;
  TopoNavNode::AdjacentNodes candidate_nodes = nodes_[associated_node_]->getAdjacentNodeIDs();
  candidate_nodes.push_back(associated_node_); //all adjacent nodes and current node
  double dist_metric_tmp;
  double dist_metric_min = DBL_MAX;
  for (int i = 0; i < candidate_nodes.size(); i++) {
    dist_metric_tmp = calcDistance(robot_pose_tf_, nodes_[candidate_nodes.at(i)]->getPoseInMap(tf_toponavmap2map_));
    if (dist_metric_tmp < dist_metric_min) {
      dist_metric_min = dist_metric_tmp;
      associated_node_new = candidate_nodes.at(i);
    }
  }
  associated_node_ = associated_node_new;
}

/*!
 * \brief ToponavMapTransform()
 */
void TopoNavMap::updateToponavMapTransform()
{

  /*#if DEBUG
   if (ros::Time::now().toSec() > counter_*5){
   tf_toponavmap2map_.setOrigin(tf::Vector3(0.03 * ros::Time::now().toSec(), -0.05 * ros::Time::now().toSec(), 0.0));
   tf::Quaternion q;
   q.setRPY(0, 0, 0.05 * ros::Time::now().toSec());
   tf_toponavmap2map_.setRotation(q);
   counter_++;
   }
   #endif*/

  /*
   tf_toponavmap2map_.setOrigin(tf::Vector3(1.0, -2, 0.0));
   tf::Quaternion q;
   q.setRPY(0, 0, 0);
   tf_toponavmap2map_.setRotation(q);
   */

  #if LTF_PERFECTODOM
    if (associated_node_ > 0 && node_odom_at_creation_map_.size() > 0)
      tf_toponavmap2map_ = node_odom_at_creation_map_.at(associated_node_);
  #endif

  br_.sendTransform(tf::StampedTransform(tf_toponavmap2map_, ros::Time::now(), "map", "toponav_map"));
}

/*!
 * \brief Local Costmap Callback.
 */
void TopoNavMap::lcostmapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

#if DEBUG
  /*ROS_INFO("Last lcostmapCB cycle took %.4f seconds",(ros::Time::now()-last_run_lcostmap_).toSec());
   last_run_lcostmap_ = ros::Time::now();*/
#endif

  ROS_DEBUG("Local Costmap Callback");
  local_costmap_ = *msg;
  poseMsgToTF(local_costmap_.info.origin, local_costmap_origin_tf_);
  br_.sendTransform(tf::StampedTransform(local_costmap_origin_tf_, ros::Time::now(), local_costmap_.header.frame_id, "local_costmap_origin"));
}

#if DEBUG
void TopoNavMap::initialposeCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  geometry_msgs::PoseStamped initialpose_previous;
  initialpose_previous = initialpose_;
  initialpose_.header = msg->header;
  initialpose_.pose = msg->pose.pose;
  tf::Point point1, point2;
  pointMsgToTF(initialpose_.pose.position, point1);
  pointMsgToTF(initialpose_previous.pose.position, point2);

  directNavigable(point1, point2);
}
#endif

/*!
 * \brief Publish the Topological Navigation Map.
 */
void TopoNavMap::publishTopoNavMap()
{
  ROS_DEBUG("publishTopoNavMap");
  st_topological_mapping::TopologicalNavigationMap msg_map;

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
 * \brief getCurrentPose
 */
void TopoNavMap::updateRobotPose()
{
  try
  {
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
 * \brief loadMapFromMsg
 */
void TopoNavMap::loadMapFromMsg(const st_topological_mapping::TopologicalNavigationMap &toponavmap_msg)
{
  nodes_.clear();
  edges_.clear();

  //TODO - p2 - set associated node as well!
  ROS_WARN("Load map: Associated Node is currently not set properly, however, usually NodeID 1 is good, so no problems are caused");

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
void TopoNavMap::updateMap()
{

#if DEBUG
  /*
   ROS_INFO("Last updateMap cycle took %.4f seconds",(ros::Time::now()-last_run_update_).toSec());
   last_run_update_ = ros::Time::now();
   if ((ros::Time::now()-last_run_update_).toSec() > last_run_update_max_)
   last_run_update_max_ = (ros::Time::now()-last_run_update_).toSec();
   */
#endif
  updateRobotPose();

  updateToponavMapTransform();

  if (!checkCreateNode()){
    updateAssociatedNode(); //updates are only needed if no new node was created...
  }

  publishTopoNavMap();

#if DEBUG

  /*if (ros::Time().now() > ros::Time(35) && test_executed_ == 0) { //this code is to test stuff timed...
   ROS_INFO("Starting test: Deleting node 6");
   ROS_INFO("before updates last_bgl_affecting_update_ = %.5f", last_bgl_affecting_update_.toSec());
   updateNodeBGLDetails(1);
   updateNodeBGLDetails(1);
   ROS_INFO("Actually Deleting node 6");
   ROS_INFO("before delete last_bgl_affecting_update_ = %.5f", last_bgl_affecting_update_.toSec());
   deleteNode(6);
   ROS_INFO("after delete last_bgl_affecting_update_ = %.5f", last_bgl_affecting_update_.toSec());
   updateNodeBGLDetails(1);
   test_executed_++;
   }
   if (ros::Time().now() > ros::Time(37) && test_executed_ == 1) { //this code is to test stuff timed...
   ROS_INFO("Starting test: Deleting node 2");
   ROS_INFO("Actually Deleting node 2");
   deleteNode(2);
   test_executed_++;
   }

   if (ros::Time().now() > ros::Time(39) && test_executed_ == 2) { //this code is to test stuff timed...
   ROS_INFO("Starting test: Moving node 4");
   tf::Pose tmp_pose = nodes_[4]->getPose();
   tmp_pose.getOrigin().setY(tmp_pose.getOrigin().getY() + 0.3);
   updateNodeBGLDetails(1);
   updateNodeBGLDetails(1);
   updateNodeBGLDetails(4);
   updateNodeBGLDetails(4);
   ROS_INFO("Actually Moving node 4");
   nodes_[4]->setPose(tmp_pose);
   updateNodeBGLDetails(1);
   updateNodeBGLDetails(4);
   test_executed_++;
   }*/

#endif
}

/*!
 * \brief checkCreateNewNode
 */
bool TopoNavMap::checkCreateNode()
{
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
    updateNodeBGLDetails(associated_node_);
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
void TopoNavMap::checkCreateEdges()
{
  TopoNavNode &node = (*nodes_.rbegin()->second);
  double max_topo_dist = 30; //maximal topological distance for edge creation, this is to make sure that loops aren't always closed -> as the node poses are not globally consistent defined, this could otherwise result in false loop closures!
  if (getNumberOfNodes() < 2)
    return; //only continue if there are 2 or more nodes

  addEdge(node, *nodes_[associated_node_]);  //create at least edge between the new and (previous) associated_node one!

  updateNodeBGLDetails(node.getNodeID());
  TopoNavNode::DistanceBiMapNodeID dist_map = node.getDistanceMap();

  for (TopoNavNode::DistanceBiMapNodeID::right_map::const_iterator right_iter = dist_map.right.begin(); right_iter != dist_map.right.end(); right_iter++)
  {
    if (right_iter->first < max_topo_dist) {
      if (right_iter->second == node.getNodeID()) //not compare to self!
        continue;
      else if (calcDistance(node, *nodes_[right_iter->second]) > max_edge_length_) //not check if > max_edge_length_
        continue;
      else if (!edgeExists(node.getNodeID(), right_iter->second)) { //not check if already exists
        if (directNavigable(node.getPoseInMap(tf_toponavmap2map_).getOrigin(), nodes_[right_iter->second]->getPoseInMap(tf_toponavmap2map_).getOrigin())) //only create if directNavigable.
          addEdge(node, *nodes_[right_iter->second]);
      }
      //ROS_INFO("NodeID %d, has dist %.4f", right_iter->second, right_iter->first);
    }
    else
      break; //as the right version of the bimap is ordered by topo distance, we can break as soon as we have pass max_topo_dist
  }
  return;
}

#if DEPRECATED
/*!
 * \brief Returns the length of the path calculated between pose1 and pose2, based on the move_base NavFn global planner
 * \param pose1 The start pose
 * \param pose2 The end pose
 * \param length (output) The length of the path in meters
 * \return false if no path can be found
 */
bool TopoNavMap::fakePathLength(const tf::Pose &pose1, const tf::Pose &pose2, double &length)
{
  bool valid_plan = false;
  length = 0;

  geometry_msgs::PoseStamped pose1sm, pose2sm;
  pose1sm.header.frame_id = "map";
  pose2sm = pose1sm; //equal the pose msgs to have the same headers

  poseTFToMsg(pose1, pose1sm.pose);
  poseTFToMsg(pose2, pose2sm.pose);

  nav_msgs::GetPlan srv;
  std::vector<geometry_msgs::PoseStamped> path;
  srv.request.start = pose1sm;
  srv.request.goal = pose2sm;

  for (int retry = 0; retry <= 3; retry++)
  { //if it fails, try multiple times. It sometimes seems to feel because it is too busy?
    if (fakeplan_client_.call(srv) && srv.response.plan.poses.size() > 0)
    {
      path = srv.response.plan.poses;
      for (int i = 0; i < path.size() - 1; i++)
      {
        length = length + calcDistance(path.at(i).pose, path.at(i + 1).pose);
      }
      ROS_DEBUG("path.size(): %ld", path.size());
      ROS_DEBUG("Path has length=%.4fm. Start point x=%.4f,y=%.4f. End point x=%.4f,y=%.4f",
          length,
          pose1sm.pose.position.x,
          pose1sm.pose.position.y,
          pose2sm.pose.position.x,
          pose2sm.pose.position.y);
      break;
    }
    else
    {
      ROS_ERROR("fakePathLength() failed to receive a valid plan");
    }
  }
  return valid_plan;
}
#endif

/*!
 * \brief directNavigable
 */
const bool TopoNavMap::directNavigable(const tf::Point &point1,
                                       const tf::Point &point2)
{
  bool navigable = false;

  //Check if the local_costmap has changed since last run, if so, update it
  updateLCostmapMatrix();

  int line_cost;
  line_cost = getCMLineCost(point1, point2);
  ROS_DEBUG("Straight line in toponav_map from (x1,y1)=(%.4f,%.4f) to (x2,y2)=(%.4f,%.4f) has cost: %d",
            point1.getX(),
            point1.getY(),
            point2.getX(),
            point2.getY(),
            line_cost);
  // 100 is LETHAL (obstacle), 99 is INSCRIBED (will hit due to robot footprint), -1 is unknown.
  // See: http://wiki.ros.org/costmap_2d
  // See the cost_translation_table_ here: https://github.com/ros-planning/navigation/blob/hydro-devel/costmap_2d/src/costmap_2d_publisher.cpp
  if (line_cost < 90 && line_cost >= 0)
    navigable = true;

  return navigable;
}

/**\brief updates the local costmap matrix (in occupancy grid map msgs valuation).
 */
void TopoNavMap::updateLCostmapMatrix()
{
  if (local_costmap_.header.seq > costmap_lastupdate_seq_)
      { //only update the matrix if a newer version of the costmap has been published
        //set lastupdate to the current message
    costmap_lastupdate_seq_ = local_costmap_.header.seq;

    int costmap_height_c, costmap_width_c;      //_c for cells

    costmap_height_c = local_costmap_.info.height;
    costmap_width_c = local_costmap_.info.width;

    // Create the costmap as a matrix. Cost goes from 0 (free), to 100 (obstacle). -1 would be uknown, but I think it is not used for costmaps, only normal gridmaps...
    costmap_matrix_.resize(costmap_height_c, costmap_width_c);
    for (int i = 0; i < costmap_height_c; i++)
        {
      for (int j = 0; j < costmap_width_c; j++)
          {
        costmap_matrix_(i, j) = local_costmap_.data[i * costmap_width_c + j];
      }
    }
  }
}

/**\brief turn a /map tf::Point into a cell of the local costmap
 * \param map_coordinate The coordiante in the /map
 * \param cell_x (output) The girdcell coordinate x
 * \param cell_y (output) The girdcell coordinate y
 * \return N/A
 */
bool TopoNavMap::mapPoint2costmapCell(const tf::Point &map_coordinate, int &cell_i, int &cell_j) const
                                      {
  // Calculate a Transform from map coordinates to costmap origin coordinates (which is at costmap_matrix_(0,0))
  bool validpoint = true;
  std::string costmap_frame_id;      //global frame as specified in move_base local costmap params
  costmap_frame_id = local_costmap_.header.frame_id;

  tf::Stamped<tf::Point> map_coordinate_stamped(map_coordinate, ros::Time(0), "map");
  tf::Stamped<tf::Point> map_coordinate_incostmap_origin;
  try
  {
    tf_listener_.waitForTransform("local_costmap_origin", "map", ros::Time(0), ros::Duration(2.0)); //not necessary
    tf_listener_.transformPoint("local_costmap_origin", map_coordinate_stamped, map_coordinate_incostmap_origin);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  ROS_DEBUG("map_coordinate_incostmap_origin x=%.4f , y=%.4f", map_coordinate_incostmap_origin.getX(), map_coordinate_incostmap_origin.getY());

  cell_i = floor(map_coordinate_incostmap_origin.getY() / local_costmap_.info.resolution); // i is row, i.e. y
  cell_j = floor(map_coordinate_incostmap_origin.getX() / local_costmap_.info.resolution); // j is column, i.e. x

  if (cell_i < 0 || cell_j < 0 || cell_i >= local_costmap_.info.height || cell_j >= local_costmap_.info.width)
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
int TopoNavMap::getCMLineCost(const tf::Point &point1, const tf::Point &point2) const
                              {
  int cell1_i, cell1_j, cell2_i, cell2_j;
  if (mapPoint2costmapCell(point1, cell1_i, cell1_j) && mapPoint2costmapCell(point2, cell2_i, cell2_j))
    return getCMLineCost(cell1_i, cell1_j, cell2_i, cell2_j);
  else
    return INT_MAX;
}

/** \brief getCMLineCost: Find the max. cost between two points */
int TopoNavMap::getCMLineCost(const int &cell1_i, const int &cell1_j, const int &cell2_i, const int &cell2_j) const
                              {
  // This code was largely based on the CostmapModel::lineCost function
  // Defined here: http://docs.ros.org/hydro/api/base_local_planner/html/costmap__model_8cpp_source.html
  int line_cost = 0.0;
  int point_cost = -1.0;

  for (base_local_planner::LineIterator line(cell1_i, cell1_j, cell2_i, cell2_j); line.isValid(); line.advance())
      {
    point_cost = costmap_matrix_(line.getX(), line.getY()); //Score the current point

    if (point_cost < 0)
      return -1;

    if (line_cost < point_cost)
      line_cost = point_cost;
  }

  return line_cost;
}

/*!
 * \brief edgeExists
 */
const bool TopoNavMap::edgeExists(const TopoNavNode::NodeID &nodeid1, const TopoNavNode::NodeID &nodeid2) const
{
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
bool TopoNavMap::checkIsNewDoor()
{
// TODO - p3 - write this method
  ROS_WARN_ONCE(
                "Detecting/creating Doors is not yet implemented. This message will only print once.");
  return false;
}

/*!
 * \brief distanceToClosestNode
 */
double TopoNavMap::distanceToClosestNode()
{
// TODO - p3 - This method compares to all nodes -> scales poorly eventually!
// One idea to make it scale slightly better:bool anyNodeCloserThen(max_dist), which return false if there isnt any (full search space needs to be searched) or returns true if there is (usually only first part of search space needs to be searched, if you start at end of nodes_ std::map)
  double dist, minimum_dist;
  int closest_node_id;
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
        closest_node_id = it->second->getNodeID();
      }
    }
  }
  ROS_DEBUG("Minimum distance = [%f], Closest Node ID= [%d]", minimum_dist,
            closest_node_id);

  return minimum_dist;

}

/*!
 * \brief addEdge
 */
void TopoNavMap::addEdge(const TopoNavNode &start_node,
                         const TopoNavNode &end_node)
{
  new TopoNavEdge(start_node, end_node, edges_, last_bgl_affecting_update_); //Using "new", the object will not be destructed after leaving this method!
}

/*!
 * \brief addNode
 */
void TopoNavMap::addNode(const tf::Pose &pose, bool is_door, int area_id)
{
  new TopoNavNode(tf_toponavmap2map_.inverse() * pose, is_door, area_id, nodes_, last_bgl_affecting_update_); //Using "new", the object will not be destructed after leaving this method!
  #if LTF_PERFECTODOM
      tf::StampedTransform perfectodom_correction_stamped;
      tf::Transform perfectodom_correction;
      try
      {
        tf_listener_.waitForTransform("odom", "map", ros::Time(0), ros::Duration(2));
        tf_listener_.lookupTransform("odom", "map", ros::Time(0), perfectodom_correction_stamped);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("Error looking up transformation\n%s", ex.what());
      }
      perfectodom_correction = perfectodom_correction_stamped;

    node_odom_at_creation_map_[nodes_.rbegin()->second->getNodeID()] = perfectodom_correction;
  #endif
}

/*!
 * \brief deleteEdge
 */
void TopoNavMap::deleteEdge(TopoNavEdge::EdgeID edge_id)
{
  deleteEdge(*edges_[edge_id]);

}
void TopoNavMap::deleteEdge(TopoNavEdge &edge)
{
  delete &edge;
}

/*!
 * \brief deleteNode
 */
void TopoNavMap::deleteNode(TopoNavNode::NodeID node_id)
{
  deleteNode(*nodes_[node_id]);
}

void TopoNavMap::deleteNode(TopoNavNode &node)
{
  updateNodeBGLDetails(node.getNodeID()); //to make sure connected_edges are up to date!
  TopoNavNode::AdjacentEdges connected_edges = node.getAdjacentEdgeIDs();
  for (int i = 0; i < connected_edges.size(); i++)
      {
    deleteEdge((*edges_[connected_edges.at(i)]));
  }
  #if LTF_PERFECTODOM
    node_odom_at_creation_map_.erase(node.getNodeID());
  #endif
  delete &node;
}

void TopoNavMap::updateNodeBGLDetails(TopoNavNode::NodeID node_id)
{
  st_bgl::updateNodeDetails(nodes_, edges_, node_id, last_bgl_affecting_update_);
}

#if DEPRECATED
void TopoNavMap::deleteNode_old(TopoNavNode &node)
{

  TopoNavEdge::EdgeMap connected_edges = connectedEdges(node);
  for (TopoNavEdge::EdgeMap::iterator it = connected_edges.begin(); it != connected_edges.end(); it++)
  {
    deleteEdge((*it->second));
  }
  delete &node;
}
#endif

#if DEPRECATED
TopoNavEdge::EdgeMap TopoNavMap::connectedEdges(const TopoNavNode &node) const
{ //TODO - p3 - scales poorly: all edges are checked!
  TopoNavEdge::EdgeMap connected_edges;
  for (TopoNavEdge::EdgeMap::const_iterator it = edges_.begin(); it != edges_.end(); it++)
  {
    if (it->second->getStartNode().getNodeID() == node.getNodeID()
        || it->second->getEndNode().getNodeID() == node.getNodeID())
    {
      connected_edges[it->second->getEdgeID()] = (it->second);
    }
  }
  return connected_edges;
}
#endif

void TopoNavMap::nodeFromRosMsg(const st_topological_mapping::TopoNavNodeMsg &node_msg)
{
  tf::Pose tfpose;
  poseMsgToTF(node_msg.pose, tfpose);

  new TopoNavNode(node_msg.node_id, //node_id
      node_msg.last_updated, //last_updated
      node_msg.last_pose_updated, //last_pose_updated
      ros::WallTime(0), //last_bgl_update set to 0, which will make any consulting trigger update!
      tfpose, //pose
      node_msg.is_door, //is_door
      node_msg.area_id, //area_id
      nodes_, //nodes map
      last_bgl_affecting_update_ //last_toponavmap_bgl_affecting_update
      );
}

void TopoNavMap::edgeFromRosMsg(const st_topological_mapping::TopoNavEdgeMsg &edge_msg)
{
  new TopoNavEdge(edge_msg.edge_id, //edge_id
      edge_msg.last_updated, //last_updated
      edge_msg.cost, //cost
      *nodes_[edge_msg.start_node_id], //start_node
      *nodes_[edge_msg.end_node_id], //end_node
      edges_, //edges std::map
      last_bgl_affecting_update_ //last_toponavmap_bgl_affecting_update
      );
}
st_topological_mapping::TopoNavEdgeMsg
TopoNavMap::edgeToRosMsg(TopoNavEdge *edge)
//not const, as getCost can cause update of the cost!
{
  st_topological_mapping::TopoNavEdgeMsg msg_edge;
  msg_edge.edge_id = edge->getEdgeID();
  msg_edge.last_updated = edge->getLastUpdatedTime();
  msg_edge.start_node_id = edge->getStartNode().getNodeID();
  msg_edge.end_node_id = edge->getEndNode().getNodeID();
  msg_edge.cost = edge->getCost();

  return msg_edge;
}

st_topological_mapping::TopoNavNodeMsg TopoNavMap::nodeToRosMsg(const TopoNavNode *node)
{
  st_topological_mapping::TopoNavNodeMsg msg_node;
  msg_node.node_id = node->getNodeID();
  msg_node.last_updated = node->getLastUpdatedTime();
  msg_node.last_pose_updated = node->getLastPoseUpdateTime();
  msg_node.area_id = node->getAreaID();
  poseTFToMsg(node->getPose(), msg_node.pose);
  msg_node.is_door = node->getIsDoor();

  return msg_node;
}
