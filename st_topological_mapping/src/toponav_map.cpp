/**
 * @file toponav_map
 * @brief Build and maintain the topological navigation map
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/toponav_map.h>

/*!
 * \brief Constructor.
 */
TopoNavMap::TopoNavMap(ros::NodeHandle &n) :
    n_(n),
    costmap_lastupdate_seq_(0),
    max_edge_length_(2.5),
    new_node_distance_(1.0),
    move_base_client_("move_base", true), // this way, TopoNavMap is aware of the NodeHandle of this ROS node, just as ShowTopoNavMap will be...
    associated_node_(1)
{
  ros::NodeHandle private_nh("~");
  std::string scan_topic;
  std::string local_costmap_topic;

  // Parameters initialization
  private_nh.param("scan_topic", scan_topic, std::string("scan"));
  private_nh.param("local_costmap_topic", local_costmap_topic, std::string("move_base/local_costmap/costmap"));

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
  scan_sub_ = n_.subscribe(scan_topic, 1, &TopoNavMap::laserCB, this);
  toponav_map_pub_ = private_nh.advertise<st_topological_mapping::TopologicalNavigationMap>("topological_navigation_map",1, true);

  //update the map one time, at construction. This will create the first map node.
  updateMap();
  ROS_INFO("TopoNavMap object is initialized");

#if DEBUG
  test_executed_ = 0;
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
void TopoNavMap::laserCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  laser_scan_ = *msg;
  ROS_DEBUG("angle_max=%f", laser_scan_.angle_max); // to check whether it uses kinect vs hokuyo
}


/*!
 * \brief Find the node id that is currently associated with the robot. I.e. the node that the robot is currently at.
 */
void TopoNavMap::updateAssociatedNode()
{
  updateAssociatedNode_method1();
  updateAssociatedNode_method2();

}

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

void TopoNavMap::updateAssociatedNode_method2()
{
  PredecessorMapNodeID predecessor_map; //std::map<nodeID,predecessor_nodeID>
  DistanceBiMapNodeID distance_map; //boost::bimap
  AdjacentNodes adjacent_nodeids_vector;
  AdjacentEdges adjacent_edgeids_vector;

  /* TODO: create functions -> find path from A to B!
  / bool findPath(predecessormap,end_node, &path, &cost)
  / bool findPath(start_node ,end_node, &path, &cost) (creates predecessor map first!)
  */

  int previous_associated_node = associated_node_;
  st_bgl::findNodeDetails(nodes_, edges_, associated_node_, predecessor_map, distance_map, adjacent_nodeids_vector, adjacent_edgeids_vector);

  /*ROS_INFO("Predecessor Map of Associated Node with ID %d:",associated_node_);
  for (PredecessorMapNodeID::const_iterator it = predecessor_map.begin(); it != predecessor_map.end(); it++){
    ROS_INFO("NodeID = %d, Predecessor NodeID = %d",it->first,it->second);
  }

  ROS_INFO("Nodes sorted by ID, with distance (of Associated Node with ID %d):",associated_node_);
  for (DistanceBiMapNodeID::left_map::const_iterator it = distance_map.left.begin(); it != distance_map.left.end(); it++){
    ROS_INFO("NodeID = %d, Distance = %.4f[m]",it->first,it->second);
  }

  ROS_INFO("Nodes by ID, sorted by distance(of Associated Node with ID %d):",associated_node_);
  for (DistanceBiMapNodeID::right_map::const_iterator it = distance_map.right.begin(); it != distance_map.right.end(); it++){
    ROS_INFO("NodeID = %d, Distance = %.4f[m]",it->second,it->first);
  }

  ROS_INFO("Adjacent Nodes (of Associated Node with ID %d):",associated_node_);
  for (AdjacentNodes::const_iterator it = adjacent_nodeids_vector.begin(); it != adjacent_nodeids_vector.end(); it++){
    ROS_INFO("NodeID = %d",*it);
  }

  ROS_INFO("Adjacent Edges (of Associated Node with ID %d):",associated_node_);
  for (AdjacentEdges::const_iterator it = adjacent_edgeids_vector.begin(); it != adjacent_edgeids_vector.end(); it++){
    ROS_INFO("EdgeID = %d",*it);
  }*/

}

/*!
 * \brief Laser Callback. Update laser_scan_
 */
void TopoNavMap::updateToponavMapTransform()
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_listener_.resolve("map"), tf_listener_.resolve("toponav_map")));
}

/*!
 * \brief Local Costmap Callback. Update laser_scan_
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
void TopoNavMap::getCurrentPose()
{
  try
  {
    tf_listener_.waitForTransform("map", "base_link", ros::Time(0),
                                  ros::Duration(10));
    tf_listener_.lookupTransform("map", "base_link", ros::Time(0),
                                 robot_transform_tf_);
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
void TopoNavMap::loadMapFromMsg(
                                const st_topological_mapping::TopologicalNavigationMap &toponavmap_msg)
{
  nodes_.clear();
  edges_.clear();

  //TODO: set associated node as well!!!!
  ROS_WARN("Load map: Associated Node is currently not set properly, however, usually NodeID 1 is good, so no problems are caused");

  for (int i = 0; i < toponavmap_msg.nodes.size(); i++)
  {
    nodeFromRosMsg(toponavmap_msg.nodes.at(i), nodes_);
    ROS_DEBUG("Loaded node with ID=%lu to std::map nodes_", toponavmap_msg.nodes.at(i).node_id);
  }
  for (int i = 0; i < toponavmap_msg.edges.size(); i++)
  {
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
  updateToponavMapTransform();

  getCurrentPose();

  checkCreateNode();

  updateAssociatedNode();

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
   deleteNode(1);
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
 * \brief checkCreateNewNode
 */
bool TopoNavMap::checkCreateNode()
{
  int number_of_nodes = getNumberOfNodes();
  int area_id = 1; //FIXME - p2 - room_id is always 1!
  bool create_node = false;
  bool is_door = false;

  if (checkIsNewDoor())
  {
    //TODO - p3 - later, maybe door nodes should not influence other nodes. Maybe they should not be regular nodes at all. Check SAS10 for comparison.
    create_node = true;
    is_door = true;
  }
  else if (distanceToClosestNode() > new_node_distance_)
  {
    create_node = true;
  }
  if (create_node)
  {
    addNode(robot_pose_tf_, is_door, area_id);
    checkCreateEdges((*nodes_.rbegin()->second)); //(*nodes_.rbegin()->second) should pass it the node that was just created...
    return true;
  }
  else
  {
    ROS_DEBUG("No new node created");
    return false;
  }
}

/*!
 * \brief checkCreateEdges
 */
bool TopoNavMap::checkCreateEdges(const TopoNavNode &node)
{
  //TODO - p3 - This method compares with all nodes: does not scale very well.
  bool edge_created = false;
  if (getNumberOfNodes() < 2)
    return false; //only continue if there are 2 or more nodes
  double fake_path_length;

  for (TopoNavNode::NodeMap::iterator it = nodes_.begin(); it != nodes_.end(); it++)
  {
    //Not compare with itself
    if (it->second->getNodeID() == node.getNodeID())
      continue;
    // If it is very close, do not check using directNavigable,
    // but using fakePathLength to make sure that current node is always connected with the node you came from...
    // Steering a sharp corner around a doorpost could otherwise result in orphan nodes..
    // new_node_distance_+ 0.4 is because nodes are more or less 1 meter from each other, but often is 1.1, 1.2, or even 1.3 as well, as they are created a bit too late, while movement had already happened.
    if (calcDistance(node, *it->second) < (new_node_distance_ + 0.4))
    {
      fakePathLength(it->second->getPose(), node.getPose(), fake_path_length);
      if (fake_path_length < calcDistance(node, *it->second) * 1.6)
      { // allow the curved path to be up to 1.6 times longer
        addEdge(node, *(it->second));
        edge_created = true;
      }
      continue;
    }
    //Only if it is close enough
    if (calcDistance(node, *it->second) > max_edge_length_)
      continue;
    //If it is close enough AND directNavigable, create an edge
    if (!edgeExists(node, *(it->second)) && directNavigable(node.getPose().getOrigin(), it->second->getPose().getOrigin()))
    {
      addEdge(node, *(it->second));
      edge_created = true;
    }
  }
  ROS_WARN_COND(!edge_created,
                "During this 'checkCreateEdges' call, no edge was created. This can lead to a graph that has 'subgraphs' that are completely unconnected!");
  return edge_created;
}

/*!
 * \brief Returns the length of the path calculated between pose1 and pose2, based on the move_base NavFn global planner
 * \param pose1 The start pose
 * \param pose2 The end pose
 * \param length (output) The length of the path in meters
 * \return false if no path can be found
 */
//bool TopoNavMap::fakePathDistance(const tf::Stamped<tf::Pose> &pose1, const tf::Point &pose2, double distance) {
//	return fakePathDistance()
//}
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

    int costmap_height_c, costmap_width_c; //_c for cells

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
  std::string costmap_frame_id; //global frame as specified in move_base local costmap params
  costmap_frame_id = local_costmap_.header.frame_id;

  tf::Stamped<tf::Point> map_coordinate_stamped(map_coordinate, ros::Time(0), "map");
  tf::Stamped<tf::Point> map_coordinate_incostmap_origin;
  try
  {
    //listener_.waitForTransform("local_costmap_origin", "map", ros::Time(0), ros::Duration(1.0)); //not necessary
    listener_.transformPoint("local_costmap_origin", map_coordinate_stamped, map_coordinate_incostmap_origin);
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
const bool TopoNavMap::edgeExists(const TopoNavNode &node1,
                                  const TopoNavNode &node2) const
{
  //TODO - p1 - if giving the edges and ID like the string "2to1", you will have unique IDs that are descriptive enough to facilitate edgeExists etc.
  ROS_WARN_ONCE(
                "edgeExists is not yet implemented. It should help block recreation of edges in checkCreateEdge. This goes well for new edges (there is no risk of duplicates), but triggering checkCreateEdge when updating a node for example will likely lead to duplicate edges. This message will only print once.");
  return false;
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
      dist = calcDistance(*(it->second), robot_pose_tf_);

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
  new TopoNavEdge(start_node, end_node, edges_); //Using "new", the object will not be destructed after leaving this method!
}

/*!
 * \brief addNode
 */
void TopoNavMap::addNode(const tf::Pose &pose, bool is_door, int area_id)
{
  new TopoNavNode(pose, is_door, area_id, nodes_); //Using "new", the object will not be destructed after leaving this method!
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
  TopoNavEdge::EdgeMap connected_edges = connectedEdges(node);
  for (TopoNavEdge::EdgeMap::iterator it = connected_edges.begin(); it != connected_edges.end(); it++)
  {
    deleteEdge((*it->second));
  }
  delete &node;
}

TopoNavEdge::EdgeMap TopoNavMap::connectedEdges(
                                                const TopoNavNode &node) const
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

void TopoNavMap::nodeFromRosMsg(const st_topological_mapping::TopoNavNodeMsg node_msg, TopoNavNode::NodeMap &nodes)
{
  tf::Pose tfpose;
  poseMsgToTF(node_msg.pose, tfpose);

  new TopoNavNode(
                  node_msg.node_id, //node_id
      node_msg.last_updated, //last_updated
      node_msg.last_pose_updated, //last_pose_updated
      tfpose, //pose
      node_msg.is_door, //is_door
      node_msg.area_id, //area_id
      nodes //nodes map
      );
}

void TopoNavMap::edgeFromRosMsg(const st_topological_mapping::TopoNavEdgeMsg edge_msg, TopoNavEdge::EdgeMap &edges)
{
  new TopoNavEdge(edge_msg.edge_id, //edge_id
      edge_msg.last_updated, //last_updated
      edge_msg.cost, //cost
      *nodes_[edge_msg.start_node_id], //start_node
      *nodes_[edge_msg.end_node_id], //end_node
      edges //edges std::map
      );
}
st_topological_mapping::TopoNavEdgeMsg TopoNavMap::edgeToRosMsg(TopoNavEdge* edge) //not const, as getCost can cause update of the cost!
{
  st_topological_mapping::TopoNavEdgeMsg msg_edge;
  msg_edge.edge_id = edge->getEdgeID();
  msg_edge.last_updated = edge->getLastUpdatedTime();
  msg_edge.start_node_id = edge->getStartNode().getNodeID();
  msg_edge.end_node_id = edge->getEndNode().getNodeID();
  msg_edge.cost = edge->getCost();

  return msg_edge;
}

st_topological_mapping::TopoNavNodeMsg TopoNavMap::nodeToRosMsg(const TopoNavNode* node)
{
  st_topological_mapping::TopoNavNodeMsg msg_node;
  msg_node.node_id = node->getNodeID();
  msg_node.last_updated = node->getLastUpdatedTime();
  msg_node.area_id = node->getAreaID();
  poseTFToMsg(node->getPose(), msg_node.pose);
  msg_node.is_door = node->getIsDoor();

  return msg_node;
}
