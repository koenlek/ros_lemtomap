/**
 * @file save_map.cpp
 * @brief This file defines the load_map node of the st_map_server package.
 * @author Koen Lekkerkerker
 */

#include <st_map_server/save_map.h>

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

/*!
 * StMapSaver constructor
 */
StMapSaver::StMapSaver() :
    saved_map_(false)
{
  ros::NodeHandle n;
  toponav_map_topic_="topological_navigation_mapper/topological_navigation_map";
  ROS_INFO("Waiting for the toponavmap at topic: '%s'",toponav_map_topic_.c_str());
  toponavmap_sub_ = n.subscribe(toponav_map_topic_, 1, &StMapSaver::toponavmapCallback, this);
}

/*!
 * toponavmapCallback
 */
void StMapSaver::toponavmapCallback(const st_topological_mapping::TopologicalNavigationMapPtr& toponav_map_ptr)
{
  ROS_INFO("Received map with %lu nodes and %lu edges",toponav_map_ptr->nodes.size(),toponav_map_ptr->edges.size());

  std::string topnavmapdata_file = mapname_ + ".yaml";
  ROS_INFO("Writing map occupancy data to %s", topnavmapdata_file.c_str());
  FILE* topnavmapdata_yaml = fopen(topnavmapdata_file.c_str(), "w");
  if (!topnavmapdata_yaml)
  {
    ROS_ERROR("Couldn't save map file to %s", topnavmapdata_file.c_str());
    return;
  }

  fprintf(topnavmapdata_yaml,
          "#nodes: %lu \n#edges: %lu \n\n",toponav_map_ptr->nodes.size(),toponav_map_ptr->edges.size());
  fclose(topnavmapdata_yaml);

  ROS_INFO("Done saving\n");
  saved_map_ = true;
}

#define USAGE "Usage: \n" \
" save_map -h\n"\
" save_map -l to save locally, i.e. to current folder (otherwise saves to st_map_server/map)"\
" save_map -f <map name> (stores all files in folder called 'map name')"

/*!
 * Main
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "st_save_map");
  std::string mapname = "toponav_map";

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  StMapSaver map_saver;

  while (!map_saver.saved_map_)
    ros::spinOnce();

  return 0;
}
