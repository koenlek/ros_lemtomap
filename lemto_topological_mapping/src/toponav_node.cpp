/**
 * @file toponav_node
 * @brief Class defining the properties of a node of a topological navigation map
 * @author Koen Lekkerkerker
 */

#include <lemto_topological_mapping/toponav_node.h>

TopoNavNode::TopoNavNode(
                         NodeID node_id,
                         ros::Time lalemto_updated,
                         ros::Time lalemto_pose_update,
                         ros::WallTime lalemto_bgl_update,
                         tf::Pose pose,
                         bool is_door,
                         int area_id,
                         NodeMap &nodes,
                         ros::WallTime &lalemto_toponavmap_bgl_affecting_update) :
    node_id_(node_id),
    lalemto_updated_(lalemto_updated),
    lalemto_pose_update_(lalemto_pose_update),
    lalemto_bgl_update_(lalemto_bgl_update),
    pose_(pose), is_door_(is_door),
    area_id_(area_id), nodes_(nodes),
    lalemto_toponavmap_bgl_affecting_update_(lalemto_toponavmap_bgl_affecting_update)
{
  if (node_id_ >= UIDGenerator_)
    UIDGenerator_ = node_id_ + 1;
  lalemto_toponavmap_bgl_affecting_update_ = ros::WallTime::now();  //TODO: actually not needed, no new/removed node without a new/removed edge!
  nodes_[node_id_] = this;
}

TopoNavNode::TopoNavNode(
                         tf::Pose pose,
                         bool is_door,
                         int area_id,
                         NodeMap &nodes,
                         ros::WallTime &lalemto_toponavmap_bgl_affecting_update) :
    pose_(pose),
    is_door_(is_door),
    area_id_(area_id),
    nodes_(nodes),
    lalemto_toponavmap_bgl_affecting_update_(lalemto_toponavmap_bgl_affecting_update)
{
  node_id_ = UIDGenerator_++;
  lalemto_updated_ = ros::Time::now();
  lalemto_pose_update_ = lalemto_updated_;
  lalemto_toponavmap_bgl_affecting_update_ = ros::WallTime::now();  //TODO: actually not needed, no new/removed node without a new/removed edge!
  ROS_DEBUG("TopoNavNode created. id= %d, pose x=%f, y=%f, theta=%f, updated at %f",
            node_id_,
            pose_.getOrigin().x(),
            pose_.getOrigin().y(),
            tf::getYaw(pose_.getRotation()),
            lalemto_updated_.toSec());
  nodes_[node_id_] = this;
}

TopoNavNode::~TopoNavNode()
{
  nodes_.erase(node_id_);
  lalemto_toponavmap_bgl_affecting_update_ = ros::WallTime::now(); //TODO: actually not needed, no new/removed node without a new/removed edge!
  ROS_INFO("Node with ID %d is destructed", node_id_); //does not print on node shutdown! therefor: std::cerr is added...
}

int TopoNavNode::UIDGenerator_ = 1; //needs to be declared here: otherwise a "Undefined reference" error will occur at compile time

