#ifndef SHORTEST_PATHS_H
#define SHORTEST_PATHS_H

//General includes
#include <utility>
#include <vector>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>

//ROS includes
#include <ros/ros.h>

//Workspace includes
#include "st_topological_mapping/TopologicalNavigationMap.h"  //Message
#include "st_topological_mapping/TopoNavEdgeMsg.h"  //Message
#include "st_topological_mapping/TopoNavNodeMsg.h"  //Message

/**
 * @file shortest_paths
 * @brief Find the shortest path between nodes of a topological navigation map
 * @author Koen Lekkerkerker
 */

namespace st_shortest_paths
{
std::vector<int> shortestPath (const st_topological_mapping::TopologicalNavigationMap &toponavmap_msg , int start_node, int end_node);

typedef float Weight;
typedef boost::property<boost::edge_weight_t, Weight> WeightProperty;
typedef boost::property<boost::vertex_name_t, std::string> NameProperty;
typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS, NameProperty, WeightProperty > Graph;
typedef std::vector<Graph::edge_descriptor> PathType;
typedef boost::graph_traits < Graph >::vertex_descriptor Vertex;
typedef boost::property_map < Graph, boost::vertex_index_t >::type IndexMap;
typedef boost::property_map < Graph, boost::vertex_name_t >::type NameMap;
typedef boost::iterator_property_map < Vertex*, IndexMap, Vertex, Vertex& > PredecessorMap;
typedef boost::iterator_property_map < Weight*, IndexMap, Weight, Weight& > DistanceMap;
} // namespace

#endif // include guard
