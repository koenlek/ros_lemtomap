/**
 * @file show_toponav_map
 * @brief Publish markers showing the Topological Navigation Map to a visualization topic for RVIZ.
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/show_toponav_map.h>

ShowTopoNavMap::ShowTopoNavMap(ros::NodeHandle &n, const std::map<node_id_int, TopoNavNode*> &nodes,
                               const std::map<edge_id_int, TopoNavEdge*> &edges) :
    n_(n), // this way, ShowTopoNavMapis aware of the NodeHandle of this ROS node, just as TopoNavMap...
    nodes_(nodes), edges_(edges)
{
  ROS_DEBUG("ShowTopoNavMap object is constructed");

  markers_pub_ = n_.advertise<visualization_msgs::MarkerArray>("toponavmap_markerarray", 1,true);

  // Set all general marker properties to a marker
  nodes_marker_.header.frame_id = "/map";
  nodes_marker_.header.stamp = ros::Time::now();
  nodes_marker_.action = visualization_msgs::Marker::ADD;
  nodes_marker_.pose.orientation.w;
  nodes_marker_.lifetime = ros::Duration(1.5); //it will take up to this much time until deleted markers will disappear...

  // Equal other markers with this 'template' marker.
  edges_marker_ = nodes_marker_;
  doors_marker_ = nodes_marker_;

  // Set marker specific properties
  nodes_marker_.ns = "nodes";
  nodes_marker_.type = visualization_msgs::Marker::CYLINDER;
  nodes_marker_.color.r = 0.0;
  nodes_marker_.color.g = 0.0;
  nodes_marker_.color.b = 1.0;
  nodes_marker_.color.a = 0.5;
  nodes_marker_.scale.x = 0.5;
  nodes_marker_.scale.y = 0.5;
  nodes_marker_.scale.z = 0.001;

  doors_marker_.ns = "doors";
  nodes_marker_.type = visualization_msgs::Marker::CYLINDER;
  doors_marker_.color.r = 1.0;
  doors_marker_.color.g = 0.0;
  doors_marker_.color.b = 0.0;
  doors_marker_.color.a = 1;
  doors_marker_.scale.x = 0.2;
  doors_marker_.scale.y = 0.2;
  doors_marker_.scale.z = 0.001;

  edges_marker_.ns = "edges";
  edges_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  edges_marker_.scale.x = 0.05;
  edges_marker_.color.r = 0.0;
  edges_marker_.color.g = 0.0;
  edges_marker_.color.b = 0.0;
  edges_marker_.color.a = 1.0;
}

void ShowTopoNavMap::updateVisualization()
{
  toponavmap_ma_.markers.clear();
  visualizeNodes();
  visualizeEdges();
  markers_pub_.publish(toponavmap_ma_);

}

void ShowTopoNavMap::visualizeNodes()
{
  //if having gcc4.7 or higher, you could use: auto it=nodes_.begin(); it!=nodes_.end(); it++
  for(std::map<node_id_int, TopoNavNode*>::const_iterator it=nodes_.begin(); it!=nodes_.end(); it++) //TODO: This visualizes every time for all nodes! Maybe only updated nodes should be "revizualized".
  {
    //Check if it is a door node
    if(it->second->getIsDoor()==true)
    {
      doors_marker_.id = it->second->getNodeID(); //it->second->getNodeID() should equal it->first
      poseTFToMsg(it->second->getPose(),doors_marker_.pose);
      toponavmap_ma_.markers.push_back(doors_marker_);
    }
    else
    {
      nodes_marker_.id = it->second->getNodeID(); // as there can only be one per ID: updated nodes are automatically moved if pose is updated, without the need to remove the old one...
      poseTFToMsg(it->second->getPose(),nodes_marker_.pose);
      toponavmap_ma_.markers.push_back(nodes_marker_);
    }
  }
}

void ShowTopoNavMap::visualizeEdges ()
{
  for(std::map<edge_id_int, TopoNavEdge*>::const_iterator it=edges_.begin(); it!=edges_.end(); it++) //TODO: This visualizes every time for all edges! Maybe only updated edges should be "revizualized".
  {
      edges_marker_.id =it->second->getEdgeID();
      edges_marker_.points.resize(2); // each line_list exits of one line

      //Source
      edges_marker_.points[0].x = it->second->getStartNode().getPose().getOrigin().getX();
      edges_marker_.points[0].y = it->second->getStartNode().getPose().getOrigin().getY();
      edges_marker_.points[0].z = 0.0f;

      //Destination
      edges_marker_.points[1].x = it->second->getEndNode().getPose().getOrigin().getX();
      edges_marker_.points[1].y = it->second->getEndNode().getPose().getOrigin().getY();
      edges_marker_.points[1].z = 0.0f;

      toponavmap_ma_.markers.push_back(edges_marker_);
  }
}
