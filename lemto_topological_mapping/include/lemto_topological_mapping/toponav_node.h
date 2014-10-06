#ifndef TOPONAV_NODE_H
#define TOPONAV_NODE_H

#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include <map>
#include <algorithm> //std::find
#include <boost/bimap.hpp>

/*
 * The TopoNavNode class can be used to create TopoNavNode objects.
 * These objects form the nodes and all their properties that
 * together with the edges make the TopoNavMap.
 */

class TopoNavNode
{

public:
  typedef int NodeID; //This can be used to help function signatures see the difference between a node_id form any int
  typedef std::map<NodeID, TopoNavNode*> NodeMap; // ptrs are needed, as std::map makes a COPY when adding elements.

  typedef std::map<TopoNavNode::NodeID, TopoNavNode::NodeID> PredecessorMapNodeID; //parent, child, i.e node, node predecessor.
  typedef boost::bimap<TopoNavNode::NodeID, double> DistanceBiMapNodeID;
  typedef std::vector<TopoNavNode::NodeID> AdjacentNodes;
  typedef std::vector<std::string> AdjacentEdges; //TODO you cannot use TopoNavEdge::EdgeID here, because class TopoNavEdge is not included here, which is done as otherwise circular dependency issues arise

  //last_toponavmap_bgl_affecting_update needs to be in the constructors: as it is a reference to the main variable in toponav_map!
  TopoNavNode(tf::Pose pose, bool is_door, int area_id, NodeMap &nodes, ros::WallTime &last_toponavmap_bgl_affecting_update);
  TopoNavNode(NodeID node_id,
              ros::Time last_updated,
              ros::Time last_pose_update,
              ros::WallTime last_bgl_update,
              tf::Pose pose,
              bool is_door,
              int area_id,
              NodeMap &nodes,
              ros::WallTime &last_toponavmap_bgl_affecting_update
              ); // only to be used when loading map from message!

  ~TopoNavNode();

  /*
   * Public Methods
   */

  // get Methods
  const NodeID getNodeID() const
  {
    return node_id_;
  }
  const ros::Time getLastUpdatedTime() const
  {
    return last_updated_;
  }
  const ros::Time getLastPoseUpdateTime() const
  {
    return last_pose_update_;
  }
  const ros::WallTime getLastBGLUpdateTime() const
  {
    return last_bgl_update_;
  }

  const int getAreaID() const
  {
    return area_id_;
  }
  const tf::Pose getPose() const //in toponav_map frame
  {
    return pose_;
  }

  const bool getIsDoor() const
  {
    return is_door_;
  }

  const PredecessorMapNodeID getPredecessorMap() const
  {
    if (last_bgl_update_ < last_toponavmap_bgl_affecting_update_)
      ROS_ERROR("Watch out: nodes_[%d].%s was called, but has an outdated predecessor map. Please call updateNodeBGLDetails(node_id) for this node", node_id_, __FUNCTION__);
    return predecessor_map_;
  }
  const DistanceBiMapNodeID getDistanceMap() const
  {
    if (last_bgl_update_ < last_toponavmap_bgl_affecting_update_)
      ROS_ERROR("Watch out: nodes_[%d].%s was called,  but has an outdated distance map. Please call updateNodeBGLDetails(node_id) for this node", node_id_, __FUNCTION__);
    return distance_map_;
  }
  const AdjacentNodes getAdjacentNodeIDs() const
  {
    if (last_bgl_update_ < last_toponavmap_bgl_affecting_update_)
      ROS_ERROR("Watch out: nodes_[%d].%s was called, but has an outdated adjacent_nodeids_vector_. Please call updateNodeBGLDetails(node_id) for this node", node_id_, __FUNCTION__);
    return adjacent_nodeids_vector_;
  }
  const AdjacentEdges getAdjacentEdgeIDs() const
  {
    if (last_bgl_update_ < last_toponavmap_bgl_affecting_update_)
      ROS_ERROR("Watch out: nodes_[%d].%s was called, but has an outdated adjacent_edgeids_vector_. Please call updateNodeBGLDetails(node_id) for this node", node_id_, __FUNCTION__);
    return adjacent_edgeids_vector_;
  }

  // set Methods
  void setBGLNodeDetails(PredecessorMapNodeID predecessor_map, DistanceBiMapNodeID distance_map, AdjacentNodes adjacent_nodeids_vector, AdjacentEdges adjacent_edgeids_vector)
  {
    predecessor_map_ = predecessor_map;
    distance_map_ = distance_map;
    adjacent_nodeids_vector_ = adjacent_nodeids_vector;
    adjacent_edgeids_vector_ = adjacent_edgeids_vector;
    last_bgl_update_ = ros::WallTime::now();
  }

  void setAreaID(int area_id)
  {
    area_id_ = area_id;
    last_updated_ = ros::Time::now();
  }
  void setPose(tf::Pose pose)
  { //should only be used for small updates to the pose
    pose_ = pose;
    last_updated_ = ros::Time::now();
    last_pose_update_ = ros::Time::now();
  }

  void setIsDoor(bool is_door)
  {
    is_door_ = is_door;
    last_updated_ = ros::Time::now();
  }

private:
  /*
   * Variables
   */
  NodeID node_id_; //node_ids should never be changed!
  ros::Time last_updated_;
  ros::Time last_pose_update_; //used to check if edge costs need to be recalculated
  ros::WallTime last_bgl_update_; //used to check if a new bgl update is needed. Automatically updated on any BGL update to ros::WallTime::now().

  tf::Pose pose_; //TODO - p3 - It would be better to use a StampedPose, relative frames would become possible (not all will be defined in /map, but e.g. in previous frame). setPose could still be kept, not much would have to change...

  bool is_door_; /*TODO - p2 - Doors should generally be treated much differently from normal nodes, maybe it is easier and makes more sense to not define them as nodes at all, or as a derived class or something like that, otherwise, you need to put in a lot of bool checks to see everytime it if is a normal node or a door node...
   * Alternatively normal nodes can be collected in the NodeMap nodes_ while doors are collected in a DoorMap (NodeMap doors_) or so?
   */

  int area_id_; //an area is a collection of nodes, in general areas would be rooms. But in future, large spaces or outdoor spaces could be divided in smaller areas, like the coffee corner, lunch corner and sitting area in the TU Delft Aula building.
  NodeMap &nodes_;
  ros::WallTime &last_toponavmap_bgl_affecting_update_;

  //BGL variables
  PredecessorMapNodeID predecessor_map_;
  DistanceBiMapNodeID distance_map_;
  AdjacentNodes adjacent_nodeids_vector_;
  AdjacentEdges adjacent_edgeids_vector_;

  /* TODO - p3 - add some "Properties of space" msg type that I still need to implement. This should get objects, local room size (by local I mean: as measured at this node), local room shape. All defined using semantic/symbolic labels with a prob. distribution of the relevant collection of such labels (e.g. room size 0.9 large, 0.07 medium, 0.04 small).
   *  add room_type, also as a semantic/symbolic probability distribution of all possible room types.
   */

  /*
   * Private Methods
   */

protected:
  static int UIDGenerator_; //generates a unique ID for every new node.

};

#endif // TOPONAV_NODE_H

