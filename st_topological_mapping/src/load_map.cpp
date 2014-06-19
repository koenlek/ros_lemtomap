/**
 * @file load_map.cpp
 * @brief This file defines the load_map node of the st_map_server package
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/load_map.h>

/*!
 * StMapLoader constructor
 */
StMapLoader::StMapLoader(const std::string& map_fullpath) :
		map_fullpath_(map_fullpath), finished_loading_(false) {
	ros::NodeHandle n;
	toponav_map_topic_ =
			"topological_navigation_mapper/topological_navigation_map";

	if(*(map_fullpath_.end() - 1)=='/') //remove any trailing slash
		map_fullpath_.erase(map_fullpath_.end() -1);

	std::string path_to_bagfile = map_fullpath_ + "/toponav_map.bag";
	path_to_bagfile = boost::filesystem::absolute(path_to_bagfile).c_str();
	if(!boost::filesystem::exists(boost::filesystem::path(path_to_bagfile)))
		ROS_FATAL("Could not find the .bag file at:\n\t%s\nPlease verify the supplied path foldername, which was:\n\t%s",path_to_bagfile.c_str(),map_fullpath.c_str());

	/* read TopologicalNavigationMap message from .bag file */
	rosbag::Bag bagread;
	bagread.open(path_to_bagfile, rosbag::bagmode::Read);
	rosbag::View view(bagread, rosbag::TopicQuery(toponav_map_topic_));

	BOOST_FOREACH(rosbag::MessageInstance const m, view){
	toponav_map_readmsg_ =*(m.instantiate<st_topological_mapping::TopologicalNavigationMap>());
	}

	ROS_INFO("Received map with %lu nodes and %lu edges",
			toponav_map_readmsg_.nodes.size(),
			toponav_map_readmsg_.edges.size());

	bagread.close();

	ROS_INFO("Done loading\n");
	finished_loading_ = true;
}
