/**
 * @file show_toponav_map
 * @brief Publish markers showing the Topological Navigation Map to a visualization topic for RVIZ.
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/show_toponav_map.h>

//@TODO Maybe direct access to the TopoNavMap is better... Currently it uses a few links of references to end up with a read-only ref to the edges and nodes vectors.
ShowTopoNavMap::ShowTopoNavMap(ros::NodeHandle &n, const std::vector<TopoNavNode*> &nodes,
                               const std::vector<TopoNavEdge*> &edges) :
    n_(n), // this way, ShowTopoNavMapis aware of the NodeHandle of this ROS node, just as TopoNavMap...
    nodes_(nodes), edges_(edges)
{
  ros::Duration lifetime = ros::Duration(3); //it will take up to this much time until deleted markers will disappear...
  ROS_DEBUG("ShowTopoNavMap object is constructed");

  marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  nodes_marker_.header.frame_id = "/map";
  nodes_marker_.header.stamp = ros::Time::now();
  nodes_marker_.ns = "nodes";
  nodes_marker_.action = visualization_msgs::Marker::ADD;
  nodes_marker_.color.r = 0.0;
  nodes_marker_.color.g = 0.0;
  nodes_marker_.color.b = 1.0;
  nodes_marker_.color.a = 0.5;
  nodes_marker_.scale.x = 0.5;
  nodes_marker_.scale.y = 0.5;
  nodes_marker_.scale.z = 0.05;
  nodes_marker_.type = visualization_msgs::Marker::CYLINDER;
  nodes_marker_.lifetime =lifetime;

  doors_marker_.header.frame_id = "/map";
  doors_marker_.header.stamp = ros::Time::now();
  doors_marker_.ns = "doors";
  doors_marker_.action = visualization_msgs::Marker::ADD;
  doors_marker_.color.r = 1.0;     //Change color
  doors_marker_.color.g = 0.0;
  doors_marker_.color.b = 0.0;
  doors_marker_.scale.x = 0.2;     //Change dimension
  doors_marker_.scale.y = 0.2;
  doors_marker_.scale.z = 0.05;
  doors_marker_.color.a = 1;
  doors_marker_.type = visualization_msgs::Marker::CYLINDER;
  doors_marker_.lifetime =lifetime;

  edges_marker_.header.frame_id = "/map";
  edges_marker_.header.stamp = ros::Time();
  edges_marker_.ns = "marker_test_arrow_by_points";
  edges_marker_.type = visualization_msgs::Marker::ARROW;
  edges_marker_.action = visualization_msgs::Marker::ADD;
  edges_marker_.pose.orientation.x = 0.0;
  edges_marker_.pose.orientation.y = 0.0;
  edges_marker_.pose.orientation.z = 0.0;
  edges_marker_.pose.orientation.w = 1.0;
  edges_marker_.scale.x = 0.05;
  edges_marker_.scale.y = 0.05;
  edges_marker_.scale.z = 0;  //arrowhead length?
  edges_marker_.color.r = 0.0;
  edges_marker_.color.g = 0.0;
  edges_marker_.color.b = 0.0;
  edges_marker_.color.a = 1.0;
  edges_marker_.lifetime =lifetime;
}

void ShowTopoNavMap::updateVisualization()
{
  visualizeNodes();
  visualizeEdges();

  /*if (ros::Time().now()>ros::Time(5)){ //this code is to show that there is r/w access to the nodes (and edges). Which is undesired.
    ROS_INFO("nodes_.at(0) Area ID = %d",nodes_.at(0)->getAreaID());
    nodes_.at(0)->setAreaID(2);
    ROS_INFO("After setAreaID(2), nodes_.at(0) Area ID = %d",nodes_.at(0)->getAreaID());
  }*/

}

void ShowTopoNavMap::visualizeNodes()
{

  for(int i=0;i<nodes_.size();i++) //TODO: This visualizes every time for all nodes! Maybe only updated nodes should be "revizualized".
  {

    //Check if it is a door node
    if(nodes_.at(i)->getIsDoor()==true)
    {
      doors_marker_.id = nodes_.at(i)->getNodeID();
      poseTFToMsg(nodes_.at(i)->getPose(),doors_marker_.pose);
      marker_pub_.publish(doors_marker_);
    }
    else
    {
      nodes_marker_.id = nodes_.at(i)->getNodeID(); // as there can only be one per ID: updated nodes are automatically moved if pose is updated, without the need to remove the old one...
      poseTFToMsg(nodes_.at(i)->getPose(),nodes_marker_.pose);
      marker_pub_.publish(nodes_marker_);
    }
  }
}

void ShowTopoNavMap::visualizeEdges ()
{
  for(int i=0;i<edges_.size();i++) //TODO: This visualizes every time for all edges! Maybe only updated edges should be "revizualized".
  {
      edges_marker_.id = edges_.at(i)->getEdgeID();
      edges_marker_.points.resize(2);

      //Source
      edges_marker_.points[0].x = edges_.at(i)->getStartNode().getPose().getOrigin().getX();
      edges_marker_.points[0].y = edges_.at(i)->getStartNode().getPose().getOrigin().getY();
      edges_marker_.points[0].z = 0.0f;

      //Destination
      edges_marker_.points[1].x = edges_.at(i)->getEndNode().getPose().getOrigin().getX();
      edges_marker_.points[1].y = edges_.at(i)->getEndNode().getPose().getOrigin().getY();
      edges_marker_.points[1].z = 0.0f;

      marker_pub_.publish(edges_marker_);
  }
}
