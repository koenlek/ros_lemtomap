/**
 * @file show_toponav_map
 * @brief Publish markers showing the Topological Navigation Map to a visualization topic for RVIZ.
 * @author Koen Lekkerkerker
 */

#include <lemto_topological_mapping/show_toponav_map.h>

ShowTopoNavMap::ShowTopoNavMap(ros::NodeHandle &n, const TopoNavNode::NodeMap &nodes,
                               const TopoNavEdge::EdgeMap &edges,
                               const TopoNavNode::NodeID &associated_node, const double frequency) :
    n_(n), // this way, ShowTopoNavMapis aware of the NodeHandle of this ROS node, just as TopoNavMap...
    nodes_(nodes), edges_(edges), associated_node_(associated_node)
{
  ROS_DEBUG("ShowTopoNavMap object is constructed");
  ros::NodeHandle private_nh("~");

  std::string movebasetopo_feedback_topic = "move_base_topo/feedback";
  movebasetopo_feedback_sub_ = n_.subscribe(movebasetopo_feedback_topic, 1, &ShowTopoNavMap::moveBaseTopoFeedbackCB, this);

  markers_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("toponavmap_markerarray", 1, true);

  // Set all general marker properties to a marker
  nodes_marker_template_.header.frame_id = "toponav_map";
  nodes_marker_template_.header.stamp = ros::Time(); //From http://wiki.ros.org/rviz/DisplayTypes/Marker: Note that the timestamp attached to the marker message above is ros::Time(), which is time Zero (0). This is treated differently by RViz than any other time. If you use ros::Time::now() or any other non-zero value, rviz will only display the marker if that time is close enough to the current time, where "close enough" depends on TF. With time 0 however, the marker will be displayed regardless of the current time.
  nodes_marker_template_.action = visualization_msgs::Marker::ADD;
  //nodes_marker_template_.pose.orientation.w;
  nodes_marker_template_.lifetime = ros::Duration(1/frequency+0.5); //it will take up to this much time until deleted markers will disappear...
  ROS_INFO("Duration is now %.2f",1/frequency+0.5);

  // Equal other markers with this 'template' marker.
  edges_marker_template_ = nodes_marker_template_;
  doors_marker_template_ = nodes_marker_template_;

  // Set marker specific properties
  nodes_marker_template_.ns = "nodes";
  nodes_marker_template_.type = visualization_msgs::Marker::CYLINDER;
  nodes_marker_template_.color.r = 0.0;
  nodes_marker_template_.color.g = 0.0;
  nodes_marker_template_.color.b = 1.0;
  nodes_marker_template_.color.a = 0.5;
  nodes_marker_template_.scale.x = 0.5;
  nodes_marker_template_.scale.y = 0.5;
  nodes_marker_template_.scale.z = 0.1;

  doors_marker_template_.ns = "doors";
  doors_marker_template_.type = visualization_msgs::Marker::CYLINDER;
  doors_marker_template_.color.r = 1.0;
  doors_marker_template_.color.g = 0.0;
  doors_marker_template_.color.b = 0.0;
  doors_marker_template_.color.a = 1;
  doors_marker_template_.scale.x = 0.2;
  doors_marker_template_.scale.y = 0.2;
  doors_marker_template_.scale.z = 0.1;

  edges_marker_template_.ns = "edges";
  edges_marker_template_.type = visualization_msgs::Marker::LINE_STRIP;
  edges_marker_template_.scale.x = 0.05;
  edges_marker_template_.color.r = 0.0;
  edges_marker_template_.color.g = 0.0;
  edges_marker_template_.color.b = 0.0;
  edges_marker_template_.color.a = 1.0;
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
  visualization_msgs::Marker node_marker;
  visualization_msgs::Marker door_marker;

  //if having gcc4.7 or higher and enable c++11, you could use: auto it=nodes_.begin(); it!=nodes_.end(); it++
  for (TopoNavNode::NodeMap::const_iterator it = nodes_.begin(); it != nodes_.end(); it++) //TODO - p3 - This visualizes every time for all nodes! Maybe only updated nodes should be "revizualized".
  {
    node_marker = nodes_marker_template_;
    door_marker = doors_marker_template_;

    //If it is part of the navigation path: give it a nice color
    if (std::find(topo_path_nodes_.begin(), topo_path_nodes_.end(), it->second->getNodeID()) != topo_path_nodes_.end())
    {
      if (it->second->getIsDoor() == true)
      {
        door_marker.color.r = 0.4;
        door_marker.color.g = 0.0;
        door_marker.color.b = 1.0;
      }
      else
      {
        node_marker.color.r = 0.5;
        node_marker.color.g = 0.0;
        node_marker.color.b = 1.0;
        node_marker.color.a = 0.8;
      }
    }

    //If it is the associated node: give it a nice color
    if (it->second->getNodeID() == associated_node_)
    {
      if (it->second->getIsDoor() == true)
      {
        door_marker.color.r = 0.0;
        door_marker.color.g = 0.7;
        door_marker.color.b = 1.0;
      }
      else
      {
        node_marker.color.r = 0.0;
        node_marker.color.g = 0.7;
        node_marker.color.b = 1.0;
        node_marker.color.a = 0.8;
      }
    }

    //Check if it is a door node
    if (it->second->getIsDoor() == true)
    {
      door_marker.id = it->second->getNodeID(); //it->second->getNodeID() should equal it->first
      poseTFToMsg(it->second->getPose(), door_marker.pose);
      toponavmap_ma_.markers.push_back(door_marker);
    }
    else
    {
      node_marker.id = it->second->getNodeID(); // as there can only be one per ID: updated nodes are automatically moved if pose is updated, without the need to remove the old one...
      poseTFToMsg(it->second->getPose(), node_marker.pose);
      toponavmap_ma_.markers.push_back(node_marker);
    }

  }
}

void ShowTopoNavMap::visualizeEdges()
{
  for (TopoNavEdge::EdgeMap::const_iterator it = edges_.begin(); it != edges_.end(); it++) //TODO - p3 - This visualizes every time for all edges! Maybe only updated edges should be "revizualized".
  {
    visualization_msgs::Marker edge_marker = edges_marker_template_;
    if (it->second->getType()==1){
      edge_marker.ns = "edges_odom";
    } else if (it->second->getType()==2){
      edge_marker.ns = "edges_near_neighbour";
      edge_marker.scale.x = 0.03;
      edge_marker.color.a = 0.3;
    } else if (it->second->getType()==3){
      edge_marker.ns = "edges_loop_closure";
    } else{
      ROS_WARN("Uknown edge type received!");
    }
    //TODO - p2 - maybe this is actually slow and I should just accept a random unique integer (implement like how NodeIDs are generated)
    std::string edge_id_string = it->second->getEdgeID();
    boost::replace_all(edge_id_string,"to","000"); //edgeID 1to2 will become 10002, needed because of int limitation
    edge_marker.id = atoi(edge_id_string.c_str()); //cast to int

    edge_marker.points.resize(2); // each line_list exits of one line

    //Source
    edge_marker.points[0].x = it->second->getStartNode().getPose().getOrigin().getX();
    edge_marker.points[0].y = it->second->getStartNode().getPose().getOrigin().getY();
    edge_marker.points[0].z = 0.0f;

    //Destination
    edge_marker.points[1].x = it->second->getEndNode().getPose().getOrigin().getX();
    edge_marker.points[1].y = it->second->getEndNode().getPose().getOrigin().getY();
    edge_marker.points[1].z = 0.0f;

    //If it is part of the navigation path: give it a nice color
    if (std::find(topo_path_edges_.begin(), topo_path_edges_.end(), it->second->getEdgeID()) != topo_path_edges_.end())
    {
      edge_marker.color.r = 0.4;
      edge_marker.color.g = 0.0;
      edge_marker.color.b = 1.0;
    }

    toponavmap_ma_.markers.push_back(edge_marker);
  }
}

void ShowTopoNavMap::moveBaseTopoFeedbackCB (const lemto_actions::GotoNodeActionFeedback::ConstPtr &feedback)
{
  topo_path_nodes_ = feedback->feedback.route_node_ids;
  topo_path_edges_ = feedback->feedback.route_edge_ids;
}
