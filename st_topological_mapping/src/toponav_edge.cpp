/**
 * @file toponav_edge
 * @brief Class defining the properties of an edge of a topological navigation map
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/toponav_edge.h>

TopoNavEdge::TopoNavEdge(edge_id_int edge_id, ros::Time last_updated, double cost, const TopoNavNode &start_node, const TopoNavNode &end_node, std::vector<TopoNavEdge*> &edges):
edge_id_(edge_id), last_updated_(last_updated), edges_(edges), cost_(cost), start_node_(start_node), end_node_(end_node){
	if (edge_id_>=UIDGenerator_)
		UIDGenerator_=edge_id_+1;
	edges_.push_back(this);
}

TopoNavEdge::TopoNavEdge(const TopoNavNode &start_node, const TopoNavNode &end_node, std::vector<TopoNavEdge*> &edges) :
    start_node_(start_node), end_node_(end_node), edges_(edges)
{
  edge_id_=UIDGenerator_++;
  last_updated_ = ros::Time::now();
  updateCost();
  ROS_DEBUG("Edge created. id= %d from Node %d to %d, cost = %f, updated at %f",
           edge_id_,
           start_node_.getNodeID(),
           end_node_.getNodeID(), cost_,
           last_updated_.toSec());
  edges_.push_back(this);
}

TopoNavEdge::~TopoNavEdge()
{
  std::vector<TopoNavEdge*>::iterator it = std::find(edges_.begin(), edges_.end(), this);
  if (*it==this){
    ROS_DEBUG("Found Edge object with edge_id_ %d and removed it from edges_ vector", edge_id_);
    edges_.erase(it);
  }
  else{
    ROS_FATAL("Edge with ID %d could not be erased from the edges_ vector! This means there is a problem in the c++ code of this ROS node!",edge_id_);
    return;
  }
  ROS_INFO("Edge with ID %d is destructed",edge_id_); //does not print on node shutdown! therefor: std::cerr is added...
  #if DEBUG
  std::cerr << "~TopoNavEdge: Deleting edge wit ID: " << edge_id_ << std::endl;
  #endif
}


const double TopoNavEdge::updateCost()
{
  //Recalculation could also be only triggered if any of the nodes had changed. I.e. iff edge.last_updated_ < node1.last_update_ ||
  cost_ = calcDistance(start_node_, end_node_);
  return cost_;
}

int TopoNavEdge::UIDGenerator_=1; //needs to be declared here: otherwise a "Undefined reference" error will occur at compile time

