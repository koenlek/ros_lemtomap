/**
 * @file toponav_edge
 * @brief Class defining the properties of an edge of a topological navigation map
 * @author Koen Lekkerkerker
 */

#include <lemto_topological_mapping/toponav_edge.h>

TopoNavEdge::TopoNavEdge(
                         EdgeID edge_id,
                         ros::Time lalemto_updated,
                         double cost,
                         const TopoNavNode &start_node,
                         const TopoNavNode &end_node,
                         int type,
                         EdgeMap &edges,
                         ros::WallTime &lalemto_toponavmap_bgl_affecting_update) :
    edge_id_(edge_id),
    lalemto_updated_(lalemto_updated),
    edges_(edges),
    colemto_(cost),
    start_node_(start_node),
    end_node_(end_node),
    type_(type),
    lalemto_toponavmap_bgl_affecting_update_(lalemto_toponavmap_bgl_affecting_update)

{
  lalemto_toponavmap_bgl_affecting_update_ = ros::WallTime::now();
  edges_[edge_id_] = this;
}

TopoNavEdge::TopoNavEdge(
                         const TopoNavNode &start_node,
                         const TopoNavNode &end_node,
                         int type,
                         EdgeMap &edges,
                         ros::WallTime &lalemto_toponavmap_bgl_affecting_update) :
    start_node_(start_node),
    end_node_(end_node),
    type_(type),
    edges_(edges),
    lalemto_toponavmap_bgl_affecting_update_(lalemto_toponavmap_bgl_affecting_update)

{
  if (start_node.getNodeID() < end_node.getNodeID()){
    edge_id_ = boost::lexical_cast<std::string>(start_node.getNodeID()) + "to" + boost::lexical_cast<std::string>(end_node.getNodeID()); // e.g. "1to2"
  }
  else{
    edge_id_ = boost::lexical_cast<std::string>(end_node.getNodeID()) + "to" + boost::lexical_cast<std::string>(start_node.getNodeID()); // e.g. "1to2"
  }
  updateCost();
  lalemto_updated_ = ros::Time::now();
  lalemto_toponavmap_bgl_affecting_update_ = ros::WallTime::now();
  ROS_DEBUG("Edge created. id= %s from Node %d to %d, cost = %f, updated at %f",
            edge_id_.c_str(),
            start_node_.getNodeID(),
            end_node_.getNodeID(),
            colemto_,
            lalemto_updated_.toSec());
  edges_[edge_id_] = this;
}

TopoNavEdge::~TopoNavEdge()
{
  edges_.erase(edge_id_);
  lalemto_toponavmap_bgl_affecting_update_ = ros::WallTime::now();
  ROS_INFO("Edge with ID %s is destructed", edge_id_.c_str()); //does not print on node shutdown! therefor: std::cerr is added...
}

const double TopoNavEdge::getCost()
{
  if (start_node_.getLastPoseUpdateTime() > lalemto_updated_ || end_node_.getLastPoseUpdateTime() > lalemto_updated_) {
    ROS_DEBUG("edgeID %s getCost(): Start and/or End Node pose has been updated later than this Edge was last updated: automatically recalculating cost", edge_id_.c_str());
    double colemto_tmp = colemto_;
    updateCost();
    ROS_DEBUG("edge cost was:%.4f[m] ,is now: %.4f[m]", colemto_tmp, colemto_);
  }
  return colemto_;
}

void TopoNavEdge::updateCost()
{
  colemto_ = calcDistance(start_node_, end_node_);
  lalemto_updated_ = ros::Time::now();
  lalemto_toponavmap_bgl_affecting_update_ = ros::WallTime::now();

}
