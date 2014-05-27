/**
 * @file toponav_edge
 * @brief Class defining the properties of an edge of a topological navigation map
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/toponav_edge.h>

TopoNavEdge::TopoNavEdge(EdgeID edge_id, ros::Time last_updated, double cost, const TopoNavNode &start_node, const TopoNavNode &end_node, EdgeMap &edges):
edge_id_(edge_id), last_updated_(last_updated), edges_(edges), cost_(cost), start_node_(start_node), end_node_(end_node){
	if (edge_id_>=UIDGenerator_)
		UIDGenerator_=edge_id_+1;
	edges_[edge_id_]=this;
}

TopoNavEdge::TopoNavEdge(const TopoNavNode &start_node, const TopoNavNode &end_node, EdgeMap &edges) :
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
  edges_[edge_id_]=this;
}

TopoNavEdge::~TopoNavEdge()
{
  edges_.erase(edge_id_);
  ROS_INFO("Edge with ID %d is destructed",edge_id_); //does not print on node shutdown! therefor: std::cerr is added...
  #define DEBUG 0 //this has only effect for its local scope!
  #if DEBUG
  std::cerr << "~TopoNavEdge: Deleting edge with ID: " << edge_id_ << std::endl;
  #endif
}


const double TopoNavEdge::updateCost()
{
  //Recalculation could also be only triggered if any of the nodes had changed. I.e. iff edge.last_updated_ < node1.last_update_ ||
  cost_ = calcDistance(start_node_, end_node_);
  return cost_;
}

int TopoNavEdge::UIDGenerator_=1; //needs to be declared here: otherwise a "Undefined reference" error will occur at compile time

