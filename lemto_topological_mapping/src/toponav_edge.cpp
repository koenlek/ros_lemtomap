/**
 * @file toponav_edge
 * @brief Class defining the properties of an edge of a topological navigation map
 * @author Koen Lekkerkerker
 */

#include <lemto_topological_mapping/toponav_edge.h>

TopoNavEdge::TopoNavEdge(
                         EdgeID edge_id,
                         ros::Time last_updated,
                         double cost,
                         const TopoNavNode &start_node,
                         const TopoNavNode &end_node,
                         int type,
                         EdgeMap &edges,
                         ros::WallTime &last_toponavmap_bgl_affecting_update) :
    edge_id_(edge_id),
    last_updated_(last_updated),
    edges_(edges),
    cost_(cost),
    start_node_(start_node),
    end_node_(end_node),
    type_(type),
    last_toponavmap_bgl_affecting_update_(last_toponavmap_bgl_affecting_update)

{
  last_toponavmap_bgl_affecting_update_ = ros::WallTime::now();
  edges_[edge_id_] = this;
}

TopoNavEdge::TopoNavEdge(
                         const TopoNavNode &start_node,
                         const TopoNavNode &end_node,
                         int type,
                         EdgeMap &edges,
                         ros::WallTime &last_toponavmap_bgl_affecting_update) :
    start_node_(start_node),
    end_node_(end_node),
    type_(type),
    edges_(edges),
    last_toponavmap_bgl_affecting_update_(last_toponavmap_bgl_affecting_update)

{
  if (start_node.getNodeID() < end_node.getNodeID()){
    edge_id_ = boost::lexical_cast<std::string>(start_node.getNodeID()) + "to" + boost::lexical_cast<std::string>(end_node.getNodeID()); // e.g. "1to2"
  }
  else{
    edge_id_ = boost::lexical_cast<std::string>(end_node.getNodeID()) + "to" + boost::lexical_cast<std::string>(start_node.getNodeID()); // e.g. "1to2"
  }
  updateCost();
  last_updated_ = ros::Time::now();
  last_toponavmap_bgl_affecting_update_ = ros::WallTime::now();
  ROS_DEBUG("Edge created. id= %s from Node %d to %d, cost = %f, updated at %f",
            edge_id_.c_str(),
            start_node_.getNodeID(),
            end_node_.getNodeID(),
            cost_,
            last_updated_.toSec());
  edges_[edge_id_] = this;
}

TopoNavEdge::~TopoNavEdge()
{
  edges_.erase(edge_id_);
  last_toponavmap_bgl_affecting_update_ = ros::WallTime::now();
  ROS_INFO("Edge with ID %s is destructed", edge_id_.c_str()); //does not print on node shutdown! therefor: std::cerr is added...
}

const double TopoNavEdge::getCost()
{
  if (start_node_.getLastPoseUpdateTime() > last_updated_ || end_node_.getLastPoseUpdateTime() > last_updated_) {
    ROS_DEBUG("edgeID %s getCost(): Start and/or End Node pose has been updated later than this Edge was last updated: automatically recalculating cost", edge_id_.c_str());
    double cost_tmp = cost_;
    updateCost();
    ROS_DEBUG("edge cost was:%.4f[m] ,is now: %.4f[m]", cost_tmp, cost_);
  }
  return cost_;
}

void TopoNavEdge::updateCost()
{
  cost_ = calcDistance(start_node_, end_node_);
  last_updated_ = ros::Time::now();
  last_toponavmap_bgl_affecting_update_ = ros::WallTime::now();

}
