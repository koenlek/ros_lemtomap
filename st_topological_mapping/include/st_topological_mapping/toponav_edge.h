#ifndef TOPONAV_EDGE_H
#define TOPONAV_EDGE_H

#include "ros/ros.h"
#include "toponav_node.h"
#include "utils.h"
#include <vector>
#include <algorithm> //std::find

/*
 * The TopoNavEdge class can be used to create TopoNavEdge objects.
 * These objects form the edges that
 * together with the nodes make the TopoNavMap.
 * TODO p3 - Possibly, using a struct or so and integrate it  in TopoNavMap class instead of separate TopoNavEdge class makes more sense...
 */

typedef int edge_id_int; //This can be used to help function signatures see the difference between a edge_id form any int


class TopoNavEdge
{

public:
  TopoNavEdge(const TopoNavNode &start_node, const TopoNavNode &end_node, std::vector<TopoNavEdge*> &edges);
  ~TopoNavEdge();

  /**
   * Public Methods
   */
  const double updateCost(); //updateCosts recalcs cost when is called, to cover the case that node poses can be updated over time.

  // get Methods
  const edge_id_int getEdgeID() const { return edge_id_; } // const after the method means that the method is not allowed to change the variables of the object.
  const ros::Time getLastUpdatedTime() const { return last_updated_; }
  const double getCost() const { return cost_; }
  const TopoNavNode& getStartNode() const { return start_node_; }
  const TopoNavNode& getEndNode() const { return end_node_; }


  // set Methods
  /* There are no set methods: as node and edge ids are not to be changed,
   * there is never a need to change start and end nodes as well. Only cost
   * can be changed, as obstacles between two nodes can influence cost:
   * direct distance is thus not the best cost measure in such cases...*/
  //void setCost(double cost) { cost_=cost; last_updated_=ros::Time::now();}
  //@TODO eventually, cost should be manually changeable: in cases where euclidian distance between nodes is not the best measure for cost anymore. Please note: currently getNode() updates the cost everytime, so setting it would not work at all currently!

private:
  /**
   * Variables
   */
  edge_id_int edge_id_; //edge ids are automatically generated (starting from 1) and should never be changed!
  ros::Time last_updated_;
  double cost_;
  const TopoNavNode &start_node_; //A read only reference to the node object is created. It can use this to calculate costs all by itself.
  const TopoNavNode &end_node_;

  std::vector<TopoNavEdge*> &edges_;
  /**
   * Private Methods
   */

};

#endif //TOPONAV_EDGE_H
