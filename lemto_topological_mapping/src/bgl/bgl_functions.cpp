#include <lemto_topological_mapping/bgl/bgl_functions.h>

/**
 * @file bgl_functions
 * @brief Find details about the topological map using the boost graph library
 * @author Koen Lekkerkerker
 */

namespace lemto_bgl //boost graph library
{

/*!
 * \brief finds details about a node in the TopoNavMap, using the Boost Graph Library tools
 * \param nodes (input/output) A NodeMap containing all the TopoNavNode objects of the TopoNavMap. It also uses this to update the Node BGL details of the node_id Node.
 * \param edges A NodeMap containing all the TopoNavNode objects of the TopoNavMap
 * \param node_id The Node to be examined
 * \param last_toponavmap_bgl_affecting_update (input/output) It uses this to check if an update is needed, after each update, this is set equal to ros::WallTime::now()
 */

//TODO - p2 - It would be better to make this part of the TopoNavMap object, would remove the need to pass nodes_, edges_, and last_toponavmap_bgl_affecting_update_ parameters.

void updateNodeDetails(
                         TopoNavNode::NodeMap &nodes,
                         const TopoNavEdge::EdgeMap &edges,
                         const int node_id,
                         ros::WallTime &last_toponavmap_bgl_affecting_update
                         )
{
  if (nodes[node_id]->getLastBGLUpdateTime() >= last_toponavmap_bgl_affecting_update){
    ROS_DEBUG("NodeID %d BGL details are up to date, skipping update",node_id);
    return;
  }

  ROS_DEBUG("NodeID %d BGL details were outdated, updating now",node_id);

  TopoNavNode::PredecessorMapNodeID predecessor_map; // output, stores parents
  TopoNavNode::DistanceBiMapNodeID distance_map; // output, stores distances
  TopoNavNode::AdjacentNodes adjacent_nodeids_vector; //output, vector with adjacent nodes
  TopoNavNode::AdjacentEdges adjacent_edgeids_vector; //output, vector with adjecent edges

  /*
   * The code below is to turn the map into a format that Boost Graph can solve using Dijkstra's algorithm.
   * And to eventually turn the result into a form that can be returned to ROS again...
   * It was largely based on this example: http://programmingexamples.net/wiki/Boost/BGL/DijkstraComputePath
   */

  // Create a graph
  UndirectedGraph graph;

  int num_of_vertices = nodes.size();
  int num_of_edges = edges.size();
  bool start_node_exists = false;
  NodeID2BoostVertex vertices_boost; //maps NodeIDs to Boost Vertices and vice versa
  EdgeID2BoostEdge edges_boost; //maps EdgeIDs to Boost Edge and vice versa

  // Add vertices to boost graph
  for (TopoNavNode::NodeMap::const_iterator it = nodes.begin(); it != nodes.end(); it++)
  {
    vertices_boost.insert(NodeID2BoostVertex::value_type(it->second->getNodeID(), boost::add_vertex(graph)));

    if (it->second->getNodeID() == node_id)
    {
      start_node_exists = true;
    }
  }
  if (!start_node_exists)
  {
    ROS_FATAL_COND(!start_node_exists,
                   "The provided node_id %d does not exist in the current toponavmap msg",
                   node_id);
    ROS_FATAL("The '%s' node will now exit",
              ros::this_node::getName().c_str());
    ros::shutdown();
  }
  // Add edges to boost graph, and create a map that maps edges ids to edges
  for (TopoNavEdge::EdgeMap::const_iterator it = edges.begin(); it != edges.end(); it++)
  {
    std::pair<Edge, bool> edge_pair;
    edge_pair = boost::add_edge(
      vertices_boost.left.at(it->second->getStartNode().getNodeID()),
      vertices_boost.left.at(it->second->getEndNode().getNodeID()),
      it->second->getCost(),
      graph);
    edges_boost.insert(EdgeID2BoostEdge::value_type(it->second->getEdgeID(),edge_pair.first));
  }

  Vertex vertext_BOOST_src = vertices_boost.left.at(node_id);

  // Create things for Dijkstra
  std::vector<Vertex> predecessors(boost::num_vertices(graph)); // To store parents
  std::vector<Weight> distances(boost::num_vertices(graph)); // To store distances
  IndexMap indexMap = boost::get(boost::vertex_index, graph);
  PredecessorMap predecessorMap(&predecessors[0], indexMap);
  DistanceMap distanceMap(&distances[0], indexMap);

  /* Compute shortest paths from starting vertex to all other vertices, and store the output in predecessors and distances
   * boost::dijkstra_shortest_paths(g, vertext_BOOST_src, boost::predecessor_map(predecessorMap).distance_map(distanceMap));
   * This is exactly the same as the above line - it is the idea of "named parameters" - you can pass the
   * predecessor map and the distance map in any order.
   */

  boost::dijkstra_shortest_paths(graph, vertext_BOOST_src,
                                 boost::distance_map(distanceMap).predecessor_map(predecessorMap)); //TODO - p3 - Currently: it searches source to all, instead of source to target. Comp. time will be limited by switching to the latter.

  // Output results
  ROS_DEBUG("distances and parents:");
  BGL_FORALL_VERTICES(v, graph, UndirectedGraph){
    ROS_DEBUG("distance(NodeID %d, NodeID %d) = %.4f, predecessor NodeID %d = NodeID %d",
        vertices_boost.right.at(vertext_BOOST_src),
        vertices_boost.right.at(v),
        distanceMap[v],
        vertices_boost.right.at(v),
        vertices_boost.right.at(predecessorMap[v])
    );
    predecessor_map[vertices_boost.right.at(v)] = vertices_boost.right.at(predecessorMap[v]);
    distance_map.insert(boost::bimap<TopoNavNode::NodeID,double>::value_type(vertices_boost.right.at(v),distanceMap[v]));
  }

  typename boost::graph_traits < UndirectedGraph >::adjacency_iterator vi, vi_end;
  ROS_DEBUG("Adjacent nodes of source NodeID %d are:",vertices_boost.right.at(vertext_BOOST_src));
  for (boost::tie(vi, vi_end) = boost::adjacent_vertices(vertext_BOOST_src, graph); vi != vi_end; ++vi){
      ROS_DEBUG("\tAdjacent NodeID:%d",vertices_boost.right.at(*vi));
      adjacent_nodeids_vector.push_back(vertices_boost.right.at(*vi));
  }

  typename boost::graph_traits < UndirectedGraph >::out_edge_iterator ei, ei_end;
  ROS_DEBUG("Adjacent edges of source NodeID %d are:",vertices_boost.right.at(vertext_BOOST_src));
  for (boost::tie(ei, ei_end) = boost::out_edges(vertext_BOOST_src, graph); ei != ei_end; ++ei){
      ROS_DEBUG("\tAdjacent EgdeID:%s",edges_boost.right.at(*ei).c_str());
      adjacent_edgeids_vector.push_back(edges_boost.right.at(*ei));
  }

  //setBGLNodeDetails updates the Node its last_bgl_update, so next, make last_toponavmap_bgl_affecting_update equal to this.
  nodes[node_id]->setBGLNodeDetails(predecessor_map, distance_map, adjacent_nodeids_vector, adjacent_edgeids_vector);
  last_toponavmap_bgl_affecting_update = nodes[node_id]->getLastBGLUpdateTime();
  return;
}
}
