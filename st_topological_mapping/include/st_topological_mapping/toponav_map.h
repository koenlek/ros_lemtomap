#ifndef TOPONAV_MAP_H
#define TOPONAV_MAP_H

// General includes
#include "string"
//#include <algorithm> //std::find
#include <map>

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

  std::map<NodeID, TopoNavNode*> nodes_; // ptrs are needed, as std::map makes a COPY  when adding elements.
  std::map<EdgeID, TopoNavEdge*> edges_; //TODO: these std::maps now form the original maps, the TopoNavNode and TopoNavEdge work with references to these. Maybe it makes more sense to let them manage these maps themselves and give TopoNavMap access through a reference?

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
  void deleteEdge(EdgeID edge_id);
  void deleteEdge(TopoNavEdge &edge);
  void deleteNode(NodeID node_id);
  void deleteNode(TopoNavNode &node);

  //Get methods
  const std::map<NodeID, TopoNavNode*>& getNodes() const { return nodes_; } //TODO: The type const std::map<NodeID, TopoNavNode*>& gives r/w access to the objects where the pointers are pointing to. Const only applies to the map itself and the pointers (i.e. the pointer addresses are protected from manipulation, but not the data they are pointing at).
  const std::map<EdgeID, TopoNavEdge*>& getEdges() const { return edges_; }

  const int getNumberOfNodes() const { return nodes_.size(); } // return the number of nodes
  const int getNumberOfEdges() const { return edges_.size(); } // return the number of edges

  std::map<EdgeID, TopoNavEdge*> connectedEdges(const TopoNavNode &node) const; //returns a std::map with pointers to the edges connected to node.

  // conversions from/to ROS msgs
  void edgeFromRosMsg(const st_topological_mapping::TopoNavEdgeMsg edge_msg, std::map<EdgeID, TopoNavEdge*> &edges);
  void nodeFromRosMsg(const st_topological_mapping::TopoNavNodeMsg node_msg, std::map<NodeID, TopoNavNode*> &nodes);
  st_topological_mapping::TopoNavEdgeMsg edgeToRosMsg(const TopoNavEdge* edge);
  st_topological_mapping::TopoNavNodeMsg nodeToRosMsg(const TopoNavNode* node);
};

#endif
