#ifndef TOPONAV_EDGE_H
#define TOPONAV_EDGE_H

#include "ros/ros.h"
#include "toponav_node.h"
#include "utils.h"
#include <map>
#include <algorithm> //std::find

/*
 * The TopoNavEdge class can be used to create TopoNavEdge objects.
 * These objects form the edges that
 * together with the nodes make the TopoNavMap.
 * TODO - p3 - Possibly, using a struct or so and integrate it  in TopoNavMap class instead of separate TopoNavEdge class makes more sense...
 */

class TopoNavEdge
{
public:
  typedef std::string EdgeID; //This can be used to help function signatures see the difference between a edge_id form any string
  typedef std::map<EdgeID, TopoNavEdge*> EdgeMap;

  TopoNavEdge(const TopoNavNode &start_node,
              const TopoNavNode &end_node,
              int type,
              EdgeMap &edges,
              ros::WallTime &last_toponavmap_bgl_affecting_update);
  TopoNavEdge(EdgeID edge_id,
              ros::Time last_updated,
              double cost,
              const TopoNavNode &start_node,
              const TopoNavNode &end_node,
              int type,
              EdgeMap &edges,
              ros::WallTime &last_toponavmap_bgl_affecting_update
              ); // only to be used when loading map from message!
  ~TopoNavEdge();

  /**
   * Public Methods
   */
  void updateCost(); //updateCosts recalcs cost when is called, to cover the case that node poses can be updated over time.

  // get Methods
  const EdgeID getEdgeID() const
  {
    return edge_id_;
  } // const after the method means that the method is not allowed to change the variables of the object.
  const ros::Time getLastUpdatedTime() const
  {
    return last_updated_;
  }
  const double getCost();

  const TopoNavNode& getStartNode() const
  {
    return start_node_;
  }
  const TopoNavNode& getEndNode() const
  {
    return end_node_;
  }
  const int getType() const
    {
      return type_;
  }

  // set Methods
  /* There are no set methods: as node and edge ids are not to be changed,
   * there is never a need to change start and end nodes as well. Only cost
   * can be changed, as obstacles between two nodes can influence cost:
   * direct distance is thus not the best cost measure in such cases...*/
private:
  /**
   * Variables
   */
  EdgeID edge_id_; //edge ids are automatically generated and should never be changed!
  ros::Time last_updated_;
  double cost_;
  int type_; // type 1 is odom, 2 is near_neighbour, 3 is loop_closure //TODO - p3 - turn into some nice enum instead of a plain int...
  const TopoNavNode &start_node_; //A read only reference to the node object is created. It can use this to calculate costs all by itself.
  const TopoNavNode &end_node_;

  EdgeMap &edges_;
  ros::WallTime &last_toponavmap_bgl_affecting_update_;

  /**
   * Private Methods
   */

};

#endif //TOPONAV_EDGE_H
