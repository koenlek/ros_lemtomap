/**
 * @file load_map.cpp
 * @brief This file defines the load_map node of the st_map_server package
 * @author Koen Lekkerkerker
 */

#include <st_map_server/load_map.h>

/*!
 * StMapLoader constructor
 */
StMapLoader::StMapLoader(const std::string& mapname) :
		mapname_(mapname), finished_loading_(false) {
	ros::NodeHandle n;
	toponav_map_topic_ =
			"topological_navigation_mapper/topological_navigation_map";

	/* read TopologicalNavigationMap message from .bag file */
	rosbag::Bag bagread;
	bagread.open(mapname_ + "/" + mapname_ + ".bag", rosbag::bagmode::Read);
	rosbag::View view(bagread, rosbag::TopicQuery(toponav_map_topic_));
	st_topological_mapping::TopologicalNavigationMap toponav_map_readmsg;

	BOOST_FOREACH(rosbag::MessageInstance const m, view) {
		toponav_map_readmsg =*(m.instantiate<st_topological_mapping::TopologicalNavigationMap>());
	}

	ROS_INFO("Received map with %lu nodes and %lu edges",
	toponav_map_readmsg.nodes.size(),
	toponav_map_readmsg.edges.size());

	bagread.close();

	ROS_INFO("Done loading\n");
	finished_loading_=true;
}

#define USAGE "Usage: \n" \
" save_map -h\n"\
" save_map -l to save locally, i.e. to current folder (otherwise saves to st_map_server/map)\n"\
" save_map -f <map_name> (stores all files in folder called 'map name')"

/*!
 * Main
 */

int main(int argc, char** argv) {
	ros::init(argc, argv, "st_load_map");
	std::string mapname = "toponav_map";

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-h")) {
			puts(USAGE);
			return 0;
		} else if (!strcmp(argv[i], "-f")) {
			if (++i < argc) {
				mapname = argv[i];
				ROS_INFO("-f received, with '%s'", argv[i]);
			} else {
				puts(USAGE);
				return 1;
			}
		} else {
			puts(USAGE);
			return 1;
		}
	}

	StMapLoader map_loader(mapname);

	while (!map_loader.finished_loading_){
		ros::spinOnce();
	}

	return 0;
}
