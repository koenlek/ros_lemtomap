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
} // namespace

#endif // include guard
