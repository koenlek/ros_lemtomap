#ifndef TOPONAV_MAP_H
#define TOPONAV_MAP_H

// General includes
#include "string"
#include <math.h> //floor
#include <algorithm> //min
#include <map>
#include <Eigen/Dense>

#include <boost/bimap.hpp>

// ROS includes
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <base_local_planner/line_iterator.h> //Use this to find the cost of a line. Although it is meant to be used in a base_local_planner context, it is also suitable to check if an edge should be created
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h> //service

#include <ecl/time/cpuwatch.hpp>
#include <ecl/time/time_data.hpp>

// Local includes
#include "toponav_node.h"
#include "toponav_edge.h"
#include "utils.h"
#include "st_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "st_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "st_topological_mapping/TopoNavNodeMsg.h"  //Message
#include "st_topological_mapping/bgl/bgl_functions.h"

/*
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

  sensor_msgs::LaserScan laser_scan_; //stores robots current laser scans
  nav_msgs::OccupancyGrid local_costmap_;

  ros::Publisher toponav_map_pub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber local_costmap_sub_;

  unsigned int costmap_lastupdate_seq_;
  Eigen::MatrixXi costmap_matrix_;
  double max_edge_length_;
  double new_node_distance_;

  tf::Transform local_costmap_origin_tf_;
  tf::TransformBroadcaster br_;
  tf::TransformListener tf_listener_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
  ros::ServiceClient fakeplan_client_;

  #if DEBUG
    int test_executed_;

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
  void laserCB(const sensor_msgs::LaserScan::ConstPtr &msg); //This could be used for door detection
  void lcostmapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void getCurrentPose(); // get current pose
  void publishTopoNavMap(); //publish the full map to a msg
  void updateLCostmapMatrix(); //update the matrix that has the local costmap in it.
  bool fakePathLength(const tf::Pose &pose1, const tf::Pose &pose2, double &length);

  void updateToponavMapTransform();
  void updateAssociatedNode(); // return the node_id where the robot is currently at.
  void updateAssociatedNode_method1();
  void updateAssociatedNode_method2();
  void updateNodeBGLDetails(TopoNavNode::NodeID node_id); // a handy shorthand for the otherwise long "st_bgl::updateNodeDetails" function

  bool mapPoint2costmapCell(const tf::Point &map_coordinate, int &cell_i, int &cell_j) const; //convert a point in /map to a cell in the local costmap
  int getCMLineCost(const int &cell1_i, const int &cell1_j, const int &cell2_i, const int &cell2_j) const;
  int getCMLineCost(const tf::Point &point1, const tf::Point &point2) const;

  bool checkCreateNode(); //Checks if a new nodes should be created and creates it when needed. Also checks for doors to add new doors nodes and creates edges for the new node when possible.
  bool checkCreateEdges(const TopoNavNode &node); //Checks if an edge can be created between node n and any other nodes. Creates it when possible.
  bool checkIsNewDoor(); //Checks if a there is a new door
  const bool directNavigable(const tf::Point &point1,
                             const tf::Point &point2); //This method checks whether there is nothing (objects/walls) blocking the direct route between point1 and point2

  const bool edgeExists(const TopoNavNode &node1,
                        const TopoNavNode &node2) const;

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
  void loadMapFromMsg(
                      const st_topological_mapping::TopologicalNavigationMap &toponavmap_msg); //should only be used to pre-load a map at the start of this ROS nodes lifetime.

  // these are the preferred functions to add/delete nodes/edges: do not try to add/delete them in another way!
  void addEdge(const TopoNavNode &start_node, const TopoNavNode &end_node);
  void addNode(const tf::Pose &pose, bool is_door, int area_id);
  void deleteEdge(TopoNavEdge::EdgeID edge_id);
  void deleteEdge(TopoNavEdge &edge);
  void deleteNode(TopoNavNode::NodeID node_id);
  void deleteNode(TopoNavNode &node);

  #if DEPRECATED //these are likely to be removed later on
    TopoNavEdge::EdgeMap connectedEdges(const TopoNavNode &node) const; //returns a std::map with pointers to the edges connected to node.
    void deleteNode_old(TopoNavNode &node);
  #endif

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
  void edgeFromRosMsg(const st_topological_mapping::TopoNavEdgeMsg edge_msg);
  void nodeFromRosMsg(const st_topological_mapping::TopoNavNodeMsg node_msg);
  st_topological_mapping::TopoNavEdgeMsg edgeToRosMsg(TopoNavEdge* edge);
  st_topological_mapping::TopoNavNodeMsg nodeToRosMsg(const TopoNavNode* node);
};

#endif
