/**
 * @file toponav_node
 * @brief Class defining the properties of a node of a topological navigation map
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/toponav_node.h>

TopoNavNode::TopoNavNode(node_id_int node_id, ros::Time last_updated, tf::Pose pose, bool is_door, int area_id, std::vector<TopoNavNode*> &nodes):
node_id_(node_id), last_updated_(last_updated), pose_(pose),is_door_(is_door),area_id_(area_id), nodes_(nodes){
	if (node_id_>=UIDGenerator_)
		UIDGenerator_=node_id_+1;
	nodes_.push_back(this);
}

TopoNavNode::TopoNavNode(tf::Pose pose, bool is_door, int area_id, std::vector<TopoNavNode*> &nodes):
pose_(pose),is_door_(is_door),area_id_(area_id),nodes_(nodes)
{
  node_id_=UIDGenerator_++;
  last_updated_=ros::Time::now();
  ROS_DEBUG("TopoNavNode created. id= %d, pose x=%f, y=%f, theta=%f, updated at %f",
            node_id_,
            pose_.getOrigin().x(),
            pose_.getOrigin().y(),
            tf::getYaw(pose_.getRotation()),
            last_updated_.toSec());
  nodes_.push_back(this);
}

TopoNavNode::~TopoNavNode()
{
  std::vector<TopoNavNode*>::iterator it = std::find(nodes_.begin(), nodes_.end(), this);
  if (*it==this){
    ROS_DEBUG("Found Node object with node_id_ %d and removed it from nodes_ vector", node_id_);
    nodes_.erase(it);
  }
  else{
    ROS_FATAL("Node with ID %d could not be erased from the nodes_ vector! This means there is a problem in the c++ code of this ROS node!",node_id_);
    return;
  }
  ROS_INFO("Node with ID %d is destructed",node_id_);
  #if DEBUG
  std::cerr << "~TopoNavNode: Deleting node wit ID: " << node_id_ << std::endl;
  #endif
}

int TopoNavNode::UIDGenerator_=1; //needs to be declared here: otherwise a "Undefined reference" error will occur at compile time

