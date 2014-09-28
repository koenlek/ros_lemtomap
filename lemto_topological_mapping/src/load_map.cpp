/**
 * @file load_map.cpp
 * @brief This file defines the load_map node of the lemto_map_server package
 * @author Koen Lekkerkerker
 */

#include <lemto_topological_mapping/load_map.h>

/*!
 * StMapLoader constructor
 */

StMapLoader::StMapLoader(const std::string& map_fullpath) :
    map_fullpath_(map_fullpath), finished_loading_(false) {
  ros::NodeHandle n;
  toponav_map_topic_ = "topological_navigation_mapper/topological_navigation_map";

  if (*(map_fullpath_.end() - 1) == '/') //remove any trailing slash
    map_fullpath_.erase(map_fullpath_.end() - 1);

  loadBag();
  loadYAML();

  ROS_INFO("Done loading\n");
  finished_loading_ = true;
}

void StMapLoader::loadBag() {
  /* read TopologicalNavigationMap message from .bag file */
  std::string path_to_bagfile = map_fullpath_ + "/toponav_map.bag";
  path_to_bagfile = boost::filesystem::absolute(path_to_bagfile).c_str();
  if (!boost::filesystem::exists(boost::filesystem::path(path_to_bagfile)))
    ROS_FATAL("Could not find the .bag file at:\n\t%s\nPlease verify the supplied path foldername, which was:\n\t%s", path_to_bagfile.c_str(), map_fullpath_.c_str());

  rosbag::Bag bagread;
  bagread.open(path_to_bagfile, rosbag::bagmode::Read);
  rosbag::View view(bagread, rosbag::TopicQuery(toponav_map_topic_));

  BOOST_FOREACH(rosbag::MessageInstance const m, view){
  toponav_map_readmsg_ =*(m.instantiate<lemto_topological_mapping::TopologicalNavigationMap>());
}

  ROS_INFO("Received map with %lu nodes and %lu edges",
           toponav_map_readmsg_.nodes.size(),
           toponav_map_readmsg_.edges.size());

  bagread.close();
}

void StMapLoader::loadYAML() {
  std::string path_to_yamlfile = map_fullpath_ + "/toponav_map_metadata.yaml";

  YAML::Node doc = YAML::LoadFile(path_to_yamlfile.c_str());

  associated_node_ = doc[0]["associated_node"].as<int>();
  doc[1]["pose"] >> robot_pose_;

  //ROS_INFO_STREAM("associated_node = " << associated_node_);
  //ROS_INFO_STREAM("robot_pose_.position.x = " << robot_pose_.position.x);

  // todo - p3 - also load map save time (either sim time or real time, plus a way to make clear if it was a sim or real time experiment).
}

void operator >> (const YAML::Node& node, geometry_msgs::Pose& robot_pose) {
   robot_pose.position.x = node["position"]["x"].as<double>();
   robot_pose.position.y = node["position"]["y"].as<double>();
   robot_pose.position.z = node["position"]["z"].as<double>();
   robot_pose.orientation.x = node["orientation"]["x"].as<double>();
   robot_pose.orientation.y = node["orientation"]["y"].as<double>();
   robot_pose.orientation.z = node["orientation"]["z"].as<double>();
   robot_pose.orientation.w = node["orientation"]["w"].as<double>();
}

