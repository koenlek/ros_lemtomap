/**
 * @file save_map.cpp
 * @brief This file defines the load_map node of the st_map_server package.
 * @author Koen Lekkerkerker
 */

#include <st_map_server/save_map.h>

/*!
 * StMapSaver constructor
 */
StMapSaver::StMapSaver(const std::string& mapname) :
		saved_map_(false), mapname_(mapname), callback_started_(false) {
	ros::NodeHandle n;
	toponav_map_topic_ =
			"topological_navigation_mapper/topological_navigation_map";
	ROS_INFO("Waiting for the toponavmap at topic: '%s'",
			toponav_map_topic_.c_str());
	toponavmap_sub_ = n.subscribe(toponav_map_topic_, 1,
			&StMapSaver::toponavmapCallback, this);
}

/*!
 * toponavmapCallback
 */
void StMapSaver::toponavmapCallback(const st_topological_mapping::TopologicalNavigationMapConstPtr& toponav_map_ptr)
{
	callback_started_=true;
	ROS_INFO("Received map with %lu nodes and %lu edges",
			toponav_map_ptr->nodes.size(), toponav_map_ptr->edges.size());

	boost::filesystem::create_directories(mapname_);

	/* write metadata to .yaml file */
	std::string topnavmapmetadata_file = "toponav_map_metadata.yaml";
	FILE* topnavmap_metadata_yaml = fopen(std::string(mapname_ + "/" + topnavmapmetadata_file).c_str(), "w");
	if (!topnavmap_metadata_yaml) {
		ROS_ERROR("Couldn't save map file to %s",
				topnavmapmetadata_file.c_str());
		return;
	}
	fprintf(topnavmap_metadata_yaml, "#nodes: %lu \n#edges: %lu \n\n",
			toponav_map_ptr->nodes.size(), toponav_map_ptr->edges.size());
	fclose(topnavmap_metadata_yaml);

	/* write TopologicalNavigationMap message to .bag file */
	rosbag::Bag bag;
	bag.open(mapname_ + "/toponav_map.bag", rosbag::bagmode::Write);
	bag.write(toponav_map_topic_, ros::Time::now(), toponav_map_ptr);
	bag.close();

	ROS_INFO("Done saving\n");
	saved_map_ = true;
}

#define USAGE "Usage: \n" \
" save_map -h\n"\
" save_map -f <map_name> (stores all files in folder called 'map name', full path needs to be specified, ~ is not accepted for home folder...)"

/*!
 * Main
 */

int main(int argc, char** argv) {
	ros::init(argc, argv, "st_save_map");
	std::string mapname = "toponav_map";

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-h")) {
			puts(USAGE);
			return 0;
		} else if (!strcmp(argv[i], "-f")) {
			if (++i < argc) {
				mapname = argv[i];
				ROS_INFO("-f received, with '%s'", argv[i]); //TODO Implement local files (-l) as well!
			} else {
				puts(USAGE);
				return 1;
			}
		} else {
			puts(USAGE);
			return 1;
		}
	}

	StMapSaver map_saver(mapname);

	while (!map_saver.callback_started_){
		ros::spinOnce();
	}

	while (!map_saver.saved_map_){};

	return 0;
}
