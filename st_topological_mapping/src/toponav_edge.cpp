/**
 * @file toponav_edge
 * @brief Class defining the properties of an edge of a topological navigation map
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/toponav_edge.h>

TopoNavEdge::TopoNavEdge(EdgeID edge_id, ros::Time last_updated, double cost, const TopoNavNode::NodeID start_node_id, const TopoNavNode::NodeID end_node_id, EdgeMap &edges, const TopoNavNode::NodeMap &nodes):
edge_id_(edge_id), last_updated_(last_updated), edges_(edges), cost_(cost), start_node_id_(start_node_id), end_node_id_(end_node_id), nodes_(nodes){
	if (edge_id_>=UIDGenerator_)
		UIDGenerator_=edge_id_+1;
	edges_[edge_id_]=this;
}

TopoNavEdge::TopoNavEdge(const TopoNavNode::NodeID start_node_id, const TopoNavNode::NodeID end_node_id, EdgeMap &edges, const TopoNavNode::NodeMap &nodes) :
    start_node_id_(start_node_id), end_node_id_(end_node_id), edges_(edges), nodes_(nodes)
{
  edge_id_=UIDGenerator_++;
  last_updated_ = ros::Time::now();
  updateCost();
  ROS_DEBUG("Edge created. id= %d from Node %d to %d, cost = %f, updated at %f",
           edge_id_,
           start_node_id_,
           end_node_id_, cost_,
           last_updated_.toSec());
  edges_[edge_id_]=this;
}

TopoNavEdge::~TopoNavEdge()
{
  edges_.erase(edge_id_);
  ROS_INFO("Edge with ID %d is destructed",edge_id_); //does not print on node shutdown! therefor: std::cerr is added...
  #if DEBUG
  std::cerr << "~TopoNavEdge: Deleting edge with ID: " << edge_id_ << std::endl;
  #endif
}


const double TopoNavEdge::updateCost()
{
  //Recalculation could also be only triggered if any of the nodes had changed. I.e. iff edge.last_updated_ < node1.last_update_ ||
  cost_ = calcDistance(nodes_.at(start_node_id_)->getPose(), nodes_.at(end_node_id_)->getPose());
  return cost_;
}

int TopoNavEdge::UIDGenerator_=1; //needs to be declared here: otherwise a "Undefined reference" error will occur at compile time

