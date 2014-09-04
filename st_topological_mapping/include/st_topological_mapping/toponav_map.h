#ifndef TOPONAV_MAP_H
#define TOPONAV_MAP_H

#ifndef DEPRECATED
#define DEPRECATED 0
#endif

#ifndef BENCHMARKING
#define BENCHMARKING 0
#endif

// General includes
#include "string"
#include <math.h> //floor
#include <algorithm> //min
#include <iterator> //std::next
#include <map>
#include <Eigen/Dense>

#include <boost/bimap.hpp>

// ROS includes
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#if DEPRECATED
#include "sensor_msgs/LaserScan.h"
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/MapMetaData.h>
#endif
#include "nav_msgs/OccupancyGrid.h"
#include "gazebo_msgs/SetModelState.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <base_local_planner/line_iterator.h> //Use this to find the cost of a line. Although it is meant to be used in a base_local_planner context, it is also suitable to check if an edge should be created
#include <nav_msgs/GetPlan.h> //service

#include <ecl/time/cpuwatch.hpp>
#include <ecl/time/time_data.hpp>

// Local includes
#include "st_topological_mapping/toponav_node.h"
#include "st_topological_mapping/toponav_edge.h"
#include "st_topological_mapping/utils.h"
#include "st_topological_mapping/bgl/bgl_functions.h"

#include "st_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "st_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "st_topological_mapping/TopoNavNodeMsg.h"  //Message
#include "st_topological_mapping/GetAssociatedNode.h"  //Service
#include "st_topological_mapping/GetPredecessorMap.h"  //Service
#include "st_topological_mapping/IsDirectNavigable.h"  //Service

/**
 * @file toponav_map
 * @brief This class will take care of creation and maintenance of the Topological Navigation Map.
 * @author Koen Lekkerkerker
 */

class TopoNavMap
{
private:
  /**
   * Variables
   */
  ros::NodeHandle &n_;

  TopoNavNode::NodeMap nodes_;
  TopoNavEdge::EdgeMap edges_;

  //this var is auto updated by TopoNavNode and TopoNavEdge objects and is used to check if bgl update is needed (st_bgl::updateNodeDetails).
  ros::WallTime last_bgl_affecting_update_; //ros::Time() seems to update at only 100Hz (in simulation), ros::WallTime() is much more accurate

  tf::Pose robot_pose_tf_; //stores robots current pose
  TopoNavNode::NodeID associated_node_; // the node with which the robot is currently associated
  tf::StampedTransform robot_transform_tf_; //stores robots current pose as a stamped transform

#if DEPRECATED
  ros::Subscriber scan_sub_;
  sensor_msgs::LaserScan laser_scan_; //stores robots current laser scans
  ros::ServiceClient fakeplan_client_;

  ros::Subscriber global_costmap_updates_sub_;
  ros::Subscriber local_costmap_updates_sub_;
  ros::Subscriber map_metadata_sub_;
  nav_msgs::MapMetaData map_metadata_;
  std::string local_costmap_updates_topic_, global_costmap_updates_topic_;
#endif

  ros::Subscriber local_costmap_sub_;
  tf::Transform local_costmap_origin_tf_;
  nav_msgs::OccupancyGrid local_costmap_;

  ros::Subscriber global_costmap_sub_;
  tf::Transform global_costmap_origin_tf_;
  nav_msgs::OccupancyGrid global_costmap_;

  std::string local_costmap_topic_;
  std::string global_costmap_topic_;

  ros::Publisher toponav_map_pub_;
  ros::ServiceServer asso_node_servserv_;
  ros::ServiceServer predecessor_map_servserv_;
  ros::ServiceServer directnav_servserv_;

  /*
   * Note on max_edge_length_;
   *    when new edges are created, begin and end need to be in local costmap.
   *    The robot is *approx.* in the middle of the local costmap.
   *    max_edge_length_ shouldn't be much bigger than min(local costmap height, local costmap width) /2 - 0.3
   *    -0.3 is to give it some play for safety as it is only approx. in the middle.
   *    NOTE: max_edge_length_ will be neglected if max_edge_creation_ is set to true
   */
  double max_edge_length_;
  double new_node_distance_; //after how much meter a new node will be created
  double loop_closure_max_topo_dist_; //maximal topological distance for edge creation, this is to make sure that loops aren't always closed -> as the node poses are not globally consistent defined, this could otherwise result in false loop closures!

  tf::TransformBroadcaster br_;
  tf::TransformListener tf_listener_;
  tf::Transform tf_toponavmap2map_;
  bool max_edge_creation_; //if true, global costmap subscription will fire and cause maximum edges being created (which can cause some extra load). if false, the var max_edge_length_ will be used.

  #if DEBUG
    int test_executed_;
    int counter_;

    ros::WallTime last_run_lcostmap_;
    ros::WallTime last_run_update_;
    double last_run_update_max_;

    ros::Subscriber initialpose_sub_;
    geometry_msgs::PoseStamped initialpose_; //I use this to test my getCost implementation to determine if an edge is navigable
    void initialposeCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  #endif

  /**
   * Private Methods
   */
  void lcostmapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void gcostmapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  bool associatedNodeSrvCB(st_topological_mapping::GetAssociatedNode::Request &req,
                           st_topological_mapping::GetAssociatedNode::Response &res);
  bool predecessorMapSrvCB(st_topological_mapping::GetPredecessorMap::Request &req,
                           st_topological_mapping::GetPredecessorMap::Response &res);
  bool isDirectNavigableSrvCB(st_topological_mapping::IsDirectNavigable::Request &req,
                           st_topological_mapping::IsDirectNavigable::Response &res);

  void updateRobotPose(); // update robot pose to its current pose;
  void publishTopoNavMap(); //publish the full map to a msg

  void updateToponavMapTransform();
  void updateAssociatedNode(); // return the node_id where the robot is currently at.

#if DEPRECATED
  bool fakePathLength(const tf::Pose &pose1, const tf::Pose &pose2, double &length);
  void laserCB(const sensor_msgs::LaserScan::ConstPtr &msg); //This could be used for door detection

  void lcostmapUpdateCB(const map_msgs::OccupancyGridUpdate::ConstPtr &update);
  void gcostmapUpdateCB(const map_msgs::OccupancyGridUpdate::ConstPtr &update);
  void mapMetaDataCB(const nav_msgs::MapMetaData::ConstPtr &msg);
#endif

  void updateNodeBGLDetails(TopoNavNode::NodeID node_id); // a handy shorthand for the otherwise long "st_bgl::updateNodeDetails" function

  bool mapPoint2costmapCell(const tf::Point &map_coordinate, int &cell_i, int &cell_j, bool global) const; //convert a point in /map to a cell in the local or global costmap
  int getCMLineCost(const int &cell1_i, const int &cell1_j, const int &cell2_i, const int &cell2_j, bool global) const;
  int getCMLineCost(const tf::Point &point1, const tf::Point &point2, bool global) const;

  bool checkCreateNode(); //Checks if a new nodes should be created and creates it when needed. Also checks for doors to add new doors nodes and creates edges for the new node when possible.
  void checkCreateEdges(); //Checks if an edge can be created between node n and any other nodes. Creates it when possible.
  bool checkIsNewDoor(); //Checks if a there is a new door
  const bool directNavigable(const tf::Point &point1, const tf::Point &point2, bool global, int max_allowable_cost = 90); //This method checks whether there is nothing (objects/walls) blocking the direct route between point1 and point2

  const bool edgeExists(const TopoNavNode::NodeID &nodeid1, const TopoNavNode::NodeID &nodeid2) const;
  bool isInCostmap(TopoNavNode::NodeID nodeid, bool global);
  bool isInCostmap(double x, double y, bool global);

  double distanceToClosestNode(); //Checks the distance from the robot to the closest node.

public:

  //Constructor: as the second argument has a null ptr as default, the constructor serves for both TopoNavMap(n,toponavmap_msg) and TopoNavMap(n).
  TopoNavMap(ros::NodeHandle &n);
  //Destructor
  ~TopoNavMap();

  /**
   * Public Methods
   */
  // updateMap is the method that generates and maintains the topological navigation map and should be called in a (main) loop
  void updateMap();
  void loadSavedMap(const st_topological_mapping::TopologicalNavigationMap &toponavmap_msg, const int& associated_node, const geometry_msgs::Pose& robot_pose); //should only be used to pre-load a map at the start of this ROS nodes lifetime.

  // these are the preferred functions to add/delete nodes/edges: do not try to add/delete them in another way!
  void addEdge(const TopoNavNode &start_node, const TopoNavNode &end_node, int type);
  void addNode(const tf::Pose &pose, bool is_door, int area_id);
  void deleteEdge(TopoNavEdge::EdgeID edge_id);
  void deleteEdge(TopoNavEdge &edge);
  void deleteNode(TopoNavNode::NodeID node_id);
  void deleteNode(TopoNavNode &node);

  //Get methods
  const TopoNavNode::NodeMap& getNodes() const
  {
    return nodes_;
  } //TODO - p3 - gives r/w access to the objects where the pointers are pointing to -> should be read only! Const only applies to the map itself and the pointers (i.e. the pointer addresses are protected from manipulation, but not the data they are pointing at).
  const TopoNavEdge::EdgeMap& getEdges() const
  {
    return edges_;
  }
  const TopoNavNode::NodeID& getAssociatedNode() const
  {
    return associated_node_;
  }

  const int getNumberOfNodes() const
  {
    return nodes_.size();
  } // return the number of nodes
  const int getNumberOfEdges() const
  {
    return edges_.size();
  } // return the number of edges

  // conversions from/to ROS msgs
  void edgeFromRosMsg(const st_topological_mapping::TopoNavEdgeMsg &edge_msg);
  void nodeFromRosMsg(const st_topological_mapping::TopoNavNodeMsg &node_msg);
  st_topological_mapping::TopoNavEdgeMsg edgeToRosMsg(TopoNavEdge* edge);
  st_topological_mapping::TopoNavNodeMsg nodeToRosMsg(const TopoNavNode* node);
};

#endif
