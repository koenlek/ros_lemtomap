#include <st_navigation/shortest_paths.h>

/**
 * @file shortest_paths
 * @brief Find the shortest path between nodes of a topological navigation map
 * @author Koen Lekkerkerker
 */

namespace st_shortest_paths {

std::vector<int> shortestPath (const st_topological_mapping::TopologicalNavigationMap &toponavmap_msg , int start_node, int end_node)
{
	ROS_WARN_ONCE("Calculating shortest path is not yet implemented. This message will only show once\n"
			"BTW: the requested route was from node_id:%d to node_id:%d.",start_node,end_node);
	std::vector<int> v {1,2};
	return v;
}
}
