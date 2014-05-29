#include <st_navigation/shortest_paths.h>

/**
 * @file shortest_paths
 * @brief Find the shortest path between nodes of a topological navigation map
 * @author Koen Lekkerkerker
 */

namespace st_shortest_paths {

std::vector<int> shortestPath(
		st_topological_mapping::TopologicalNavigationMap toponavmap_msg,
		int start_node_id, int end_node_id) {
	ROS_INFO("Calculating shortest route from node id:%d to node id:%d.",
			start_node_id, end_node_id);

	/*
	 * The code below is to turn the map into a format that Boost Graph can solve using Dijkstra's algorithm.
	 * And to eventually turn the result into a form that can be returned to ROS again...
	 * It was largely based on this example: http://programmingexamples.net/wiki/Boost/BGL/DijkstraComputePath
	 */

	// Create a graph
	UndirectedGraph graph;

	int num_of_vertices = toponavmap_msg.nodes.size();
	int num_of_edges = toponavmap_msg.edges.size();
	bool start_node_exists = false;
	bool end_node_exists = false;
	std::vector<Vertex> vertices_boost;

	std::map<int, int> nodes_id2vecpos_map;
	std::map<int, int> edges_id2vecpos_map;

	// Add vertices to boost graph, and create a map that maps node ids to nodes
	for (int i = 0; i < num_of_vertices; i++) {
		nodes_id2vecpos_map[toponavmap_msg.nodes.at(i).node_id] = i;

		vertices_boost.push_back(boost::add_vertex(graph));
		if (toponavmap_msg.nodes.at(i).node_id == start_node_id) {
			start_node_exists = true;
			ROS_DEBUG("start_node_id %d has vecpos %d", start_node_id, i);
		}
		if (toponavmap_msg.nodes.at(i).node_id == end_node_id) {
			end_node_exists = true;
			ROS_DEBUG("end_node_id %d has vecpos %d", end_node_id, i);
		}
	}
	if (!start_node_exists || !end_node_exists) {
		ROS_FATAL_COND(!start_node_exists,
				"The provided start_node_id %d does not exist in the current toponavmap msg",
				start_node_id);
		ROS_FATAL_COND(!end_node_exists,
				"The provided end_node_id %d does not exist in the current toponavmap msg",
				end_node_id);
		ROS_FATAL("The '%s' node will now exit",
				ros::this_node::getName().c_str());
		ros::shutdown();
	}

	// Add edges to boost graph, and create a map that maps edges ids to edges
	for (int i = 0; i < num_of_edges; i++) {
		edges_id2vecpos_map[toponavmap_msg.edges.at(i).edge_id] = i;

		boost::add_edge(
				nodes_id2vecpos_map[toponavmap_msg.edges.at(i).start_node_id],
				nodes_id2vecpos_map[toponavmap_msg.edges.at(i).end_node_id],
				toponavmap_msg.edges.at(i).cost, graph);
	}

	Vertex vertext_boost_src = vertices_boost.at(
			nodes_id2vecpos_map[start_node_id]);
	Vertex vertext_boost_dest = vertices_boost.at(
			nodes_id2vecpos_map[end_node_id]);

	// Create things for Dijkstra
	std::vector<Vertex> predecessors(boost::num_vertices(graph)); // To store parents
	std::vector<Weight> distances(boost::num_vertices(graph)); // To store distances

	IndexMap indexMap = boost::get(boost::vertex_index, graph);
	PredecessorMap predecessorMap(&predecessors[0], indexMap);
	DistanceMap distanceMap(&distances[0], indexMap);

	// Compute shortest paths from starting vertex to all other vertices, and store the output in predecessors and distances
	// boost::dijkstra_shortest_paths(g, vertext_boost_src, boost::predecessor_map(predecessorMap).distance_map(distanceMap));
	// This is exactly the same as the above line - it is the idea of "named parameters" - you can pass the
	// predecessor map and the distance map in any order.

	boost::dijkstra_shortest_paths(graph, vertext_boost_src,
			boost::distance_map(distanceMap).predecessor_map(predecessorMap)); //TODO - p3 - Currently: it searches source to all, instead of source to target. Comp. time will be limited by switching to the latter.

	// Output results
	ROS_DEBUG("distances and parents:");
	BGL_FORALL_VERTICES(v, graph, UndirectedGraph){
	ROS_DEBUG("distance(NodeID %lu, NodeID %lu) = %.4f, predecessor NodeID %lu = NodeID %lu",
			toponavmap_msg.nodes.at(indexMap[vertext_boost_src]).node_id,
			toponavmap_msg.nodes.at(indexMap[v]).node_id,
			distanceMap[v],
			toponavmap_msg.nodes.at(indexMap[v]).node_id,
			toponavmap_msg.nodes.at(indexMap[predecessorMap[v]]).node_id
	);
}

	std::vector<int> path_node_id_vector;

	path_node_id_vector.push_back(
			toponavmap_msg.nodes.at(indexMap[vertext_boost_dest]).node_id);

	Vertex vertext_boost_dest_tmp = vertext_boost_dest; // We want to start at the destination and work our way back to the source
	for (Vertex vertex_boost_prev = predecessorMap[vertext_boost_dest_tmp]; // Start by setting 'u' to the destination node's predecessor
	vertex_boost_prev != vertext_boost_dest_tmp; // Keep tracking the path until we get to the source
			vertext_boost_dest_tmp = vertex_boost_prev, vertex_boost_prev =
					predecessorMap[vertext_boost_dest_tmp]) // Set the current vertex to the current predecessor, and the predecessor to one level up
			{
		path_node_id_vector.insert(path_node_id_vector.begin(),
				toponavmap_msg.nodes.at(indexMap[vertex_boost_prev]).node_id);
	}

	//turn path_node_id_vector into a string and print it
	std::stringstream path_stringstream;
	std::string path_string;
	std::copy(path_node_id_vector.begin(), path_node_id_vector.end(),
			std::ostream_iterator<int>(path_stringstream, ", "));
	path_string = path_stringstream.str();
	path_string = path_string.substr(0, path_string.size() - 2);
	ROS_INFO("\nCalculated topological path.\nNode IDs: [%s]\nTotal length: %.4f[m]",
			path_string.c_str(), distanceMap[vertext_boost_dest]);

	return path_node_id_vector;
}
}
