#include <st_navigation/shortest_paths.h>

/**
 * @file shortest_paths
 * @brief Find the shortest path between nodes of a topological navigation map
 * @author Koen Lekkerkerker
 */

namespace st_shortest_paths {

std::vector<int> shortestPath(const st_topological_mapping::TopologicalNavigationMap &toponavmap_msg, int start_node, int end_node)
{
	ROS_WARN_ONCE(
			"Calculating shortest path is not yet fully implemented. This message will only show once\n"
					"BTW: the requested route was from node_id:%d to node_id:%d.",
			start_node, end_node);

	// Create a graph
	Graph graph;

	// Add named vertices
	Vertex vertex0 = boost::add_vertex(std::string("vertex0"), graph);
	Vertex vertex1 = boost::add_vertex(std::string("vertex1"), graph);
	Vertex vertex2 = boost::add_vertex(std::string("vertex2"), graph);
	Vertex vertex3 = boost::add_vertex(std::string("vertex3"), graph);

	// Add weighted edges
	Weight weight0 = 5;
	Weight weight1 = 3;
	Weight weight2 = 2;
	Weight weight3 = 4;

	boost::add_edge(vertex0, vertex1, weight0, graph);
	boost::add_edge(vertex1, vertex3, weight1, graph);
	boost::add_edge(vertex0, vertex2, weight2, graph);
	boost::add_edge(vertex2, vertex3, weight3, graph);

	// Create things for Dijkstra
	std::vector<Vertex> predecessors(boost::num_vertices(graph)); // To store parents
	std::vector<Weight> distances(boost::num_vertices(graph)); // To store distances

	IndexMap indexMap = boost::get(boost::vertex_index, graph);
	PredecessorMap predecessorMap(&predecessors[0], indexMap);
	DistanceMap distanceMap(&distances[0], indexMap);

	// Compute shortest paths from starting vertex to all other vertices, and store the output in predecessors and distances
	// boost::dijkstra_shortest_paths(g, vertex0, boost::predecessor_map(predecessorMap).distance_map(distanceMap));
	// This is exactly the same as the above line - it is the idea of "named parameters" - you can pass the
	// predecessor map and the distance map in any order.
	boost::dijkstra_shortest_paths(graph, vertex0,
			boost::distance_map(distanceMap).predecessor_map(predecessorMap));

	// Output results
	ROS_INFO_STREAM("distances and parents:");
	NameMap nameMap = boost::get(boost::vertex_name, graph);

	BGL_FORALL_VERTICES(v, graph, Graph){
		ROS_INFO_STREAM("distance(" << nameMap[vertex0] << ", " << nameMap[v] << ") = " << distanceMap[v] << ", " <<
		  "predecessor(" << nameMap[v] << ") = " << nameMap[predecessorMap[v]]);
	}

	PathType path;
	std::vector<std::string> path_vertex_namevector; //KL

	Vertex v = vertex3; // We want to start at the destination and work our way back to the source
	path_vertex_namevector.push_back(nameMap[v]);
	for (Vertex u = predecessorMap[v]; // Start by setting 'u' to the destination node's predecessor
	u != v; // Keep tracking the path until we get to the source
			v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
			{
		std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v,
				graph);
		Graph::edge_descriptor edge = edgePair.first;

		path.push_back(edge);
		path_vertex_namevector.insert(path_vertex_namevector.begin(),
				nameMap[u]);
	}

	// Write shortest path
	ROS_INFO_STREAM("Shortest path from vertex0 to vertex3:");
	//float totalDistance = 0;
	for (PathType::reverse_iterator pathIterator = path.rbegin();
			pathIterator != path.rend(); ++pathIterator) {
		ROS_INFO_STREAM(nameMap[boost::source(*pathIterator, graph)] << " -> "
				<< nameMap[boost::target(*pathIterator, graph)] << " = "
				<< boost::get(boost::edge_weight, graph, *pathIterator));
	}

	ROS_INFO("Content of the vector path_vertex_namevector is:");

	for (unsigned int i = 0; i < path_vertex_namevector.size(); i++) {
		ROS_INFO("path_vertex_namevector.at(%d)='%s'", i,
				path_vertex_namevector.at(i).c_str());
	}

	ROS_INFO_STREAM("Distance: " << distanceMap[vertex3]);

	std::vector<int> vec { 1, 2 };
	return vec;
}
}
