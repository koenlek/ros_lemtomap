#ifndef BGL_FUNCTIONS_H
#define BGL_FUNCTIONS_H

//General includes
#include <utility>
#include <vector>
#include <map>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/bimap.hpp>

//ROS includes
#include <ros/ros.h>

//Workspace includes
#include "lemto_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "lemto_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "lemto_topological_mapping/TopoNavNodeMsg.h"  //Message
#include "lemto_topological_mapping/toponav_node.h"
#include "lemto_topological_mapping/toponav_edge.h"

/**
 * @file bgl_functions
 * @brief Find details about the topological map using the boost graph library
 * @author Koen Lekkerkerker
 */

namespace lemto_bgl
{
typedef float Weight;
typedef boost::property<boost::edge_weight_t, Weight> WeightProperty;
typedef boost::property<boost::vertex_index_t, int> IndexProperty;

typedef boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, IndexProperty, WeightProperty > UndirectedGraph;
typedef boost::graph_traits < UndirectedGraph >::vertex_descriptor Vertex;
typedef boost::graph_traits < UndirectedGraph >::edge_descriptor Edge;

typedef boost::property_map < UndirectedGraph, boost::vertex_index_t >::type IndexMap;
typedef boost::iterator_property_map < Vertex*, IndexMap, Vertex, Vertex& > PredecessorMap;
typedef boost::iterator_property_map < Weight*, IndexMap, Weight, Weight& > DistanceMap;

typedef boost::bimap< TopoNavNode::NodeID,Vertex > NodeID2BoostVertex;
typedef boost::bimap< TopoNavEdge::EdgeID,Edge > EdgeID2BoostEdge;

void updateNodeDetails(
                         TopoNavNode::NodeMap &nodes,
                         const TopoNavEdge::EdgeMap &edges,
                         const int node_id,
                         ros::WallTime &last_toponavmap_bgl_affecting_update
                         );
} // namespace

#endif // include guard
