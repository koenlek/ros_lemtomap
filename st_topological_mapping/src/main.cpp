/**
 * @file main.cpp
 * @brief This file contains the main function that forms the topological_navigation_mapper ROS node.
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/toponav_map.h>
#include <st_topological_mapping/show_toponav_map.h>
#include <st_topological_mapping/load_map.h>

/*!
 * Main
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, "topological_navigation_mapper");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	std::string load_map_path;

	private_nh.param("load_map_directory", load_map_path, std::string("")); //requires a full path to the top level directory, ~ and other env. vars are not accepted

	TopoNavMap topo_nav_map(n);
	if (load_map_path != "") {
		ROS_INFO("Map will be loaded from: %s",load_map_path.c_str());
		StMapLoader map_loader(load_map_path);
		while (!map_loader.finished_loading_) {
			ros::spinOnce();
		}
		topo_nav_map.loadMapFromMsg(
				map_loader.getTopologicalNavigationMapMsg());
	}

	ShowTopoNavMap show_topo_nav_map(n, topo_nav_map.getNodes(), topo_nav_map.getEdges());

	ros::Rate r(4);

#include <boost/preprocessor/stringize.hpp>
#ifdef CMAKE_BUILD_TYPE_DEF
	ROS_INFO("This node had CMAKE_BUILD_TYPE=%s",BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF));
#endif


	/*int cycle = 0, cycle_period = 0;
	ros::Time period(1 * 60); //1 minute
	double freq_avg, freq_cur, freq_avg_period;
	std::vector<ros::Time> benchm_update_time;
	benchm_update_time.push_back(ros::Time::now());*/

	while (n.ok()) {
		/*cycle++;
		benchm_update_time.push_back(ros::Time::now());
		freq_cur=1/(benchm_update_time.at(cycle)-benchm_update_time.at(cycle-1)).toSec();
		freq_avg=cycle/(benchm_update_time.at(cycle)-benchm_update_time.at(0)).toSec();
		ROS_INFO("Current Time: %f",ros::Time::now().toSec());
		ROS_INFO("Main cycle duration current=%.2f[Hz] avg=%.2f[Hz]",freq_cur,freq_avg);*/

		topo_nav_map.updateMap();
		show_topo_nav_map.updateVisualization();

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
