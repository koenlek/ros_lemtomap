#ifndef TOPONAV_MAP_H
#define TOPONAV_MAP_H

// General includes
#include "string"
#include <algorithm> //std::find

// ROS includes
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Pose.h>

// Local includes
#include "toponav_node.h"
#include "toponav_edge.h"
#include "utils.h"
#include "st_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "st_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "st_topological_mapping/TopoNavNodeMsg.h"  //Message

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
  std::string scan_topic_;

  std::vector<TopoNavNode*> nodes_; // ptrs are needed, as vector ALWAYS makes a COPY (passing by ref is impossible) when adding elements using e.g. nodes_.push_back().
  std::vector<TopoNavEdge*> edges_; // These are reference to these vectors of object pointers within the TopoNavMap class. Problems can arise if pointed objects are destroyed without proper updating of these vectors!

  tf::Pose robot_pose_tf_; //stores robots current pose
  tf::StampedTransform robot_transform_tf_; //stores robots current pose as a stamped transform

  sensor_msgs::LaserScan laser_scan_; //stores robots current laser scans

  ros::Publisher toponav_map_pub_;
  ros::Subscriber scan_sub_;

  tf::TransformListener tf_listener_;

  #if DEBUG
    int test_executed_;
  #endif

  /**
   * Private Methods
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg); //This could be used for door detection
  void getCurrentPose(); // get current pose
  void publishTopoNavMap(); //publish the full map to a msg

  bool checkCreateNode(); //Checks if a new nodes should be created and creates it when needed. Also checks for doors to add new doors nodes and creates edges for the new node when possible.
  bool checkCreateEdges(const TopoNavNode &node); //Checks if an edge can be created between node n and any other nodes. Creates it when possible.
  bool checkIsNewDoor(); //Checks if a there is a new door
  const bool directNavigable(const TopoNavNode &node1, const TopoNavNode &node2) const; //This method checks whether there is nothing (objects/walls) blocking the direct route between node1 and node2

  const bool edgeExists(const TopoNavNode &node1, const TopoNavNode &node2) const;

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
  void loadMapFromMsg(const st_topological_mapping::TopologicalNavigationMap &toponavmap_msg); //should only be used to pre-load a map at the start of this ROS nodes lifetime.

  // these are the preferred functions to add/delete nodes/edges: do not try to add/delete them in another way!
  void addEdge(const TopoNavNode &start_node, const TopoNavNode &end_node);
  void addNode(const tf::Pose &pose, bool is_door,int area_id);
  //void addNode(tf::Pose pose); //TODO: implement a default add node with door is false and area_id is current area_id
  void deleteEdge(edge_id_int edge_id);
  void deleteEdge(TopoNavEdge &edge);
  void deleteNode(node_id_int node_id);
  void deleteNode(TopoNavNode &node);

  //Get methods
  const std::vector<TopoNavNode*>& getNodes() const { return nodes_; } //TODO: The type const std::vector<TopoNavNode*>& gives r/w access to the objects where the pointers are pointing to. Const only applies to the vector itself and the pointers (i.e. the pointer addresses are protected from manipulation, but not the data they are pointing at).
  const std::vector<TopoNavEdge*>& getEdges() const { return edges_; }

  TopoNavNode& getNodeByID(node_id_int node_id); //return the Node that matches the supplied ID. Throw ROS_FATAL and shutdown ROS node if the node is not found!
  const int getNodeVectorPosition(const TopoNavNode &node) const; //return the position of TopoNavNode node in the nodes_ vector.
  const int getNodeVectorPositionByID(node_id_int node_id) const; //return the node vector position of the Node that matches the supplied ID. Throw ROS_FATAL and shutdown ROS node if the node is not found!
  const int getNumberOfNodes() const { return nodes_.size(); } // return the number of nodes

  TopoNavEdge& getEdgeByID(edge_id_int edge_id); //return the Edge that matches the supplied ID. Throw ROS_FATAL and shutdown ROS node if the edge is not found!
  const int getEdgeVectorPosition(const TopoNavEdge &edge) const;
  const int getEdgeVectorPositionByID(edge_id_int edge_id) const;
  const int getNumberOfEdges() const { return edges_.size(); } // return the number of edges

  std::vector<TopoNavEdge*> connectedEdges(const TopoNavNode &node) const; //returns a vector with pointers to the edges connected to node.

  // conversions from/to ROS msgs
  void edgeFromRosMsg(const st_topological_mapping::TopoNavEdgeMsg edge_msg, std::vector<TopoNavEdge*> &edges);
  void nodeFromRosMsg(const st_topological_mapping::TopoNavNodeMsg node_msg, std::vector<TopoNavNode*> &nodes);
  st_topological_mapping::TopoNavEdgeMsg edgeToRosMsg(const TopoNavEdge* edge);
  st_topological_mapping::TopoNavNodeMsg nodeToRosMsg(const TopoNavNode* node);
};

#endif
