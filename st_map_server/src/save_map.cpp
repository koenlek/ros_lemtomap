/**
 * @file save_map.cpp
 * @brief This file defines the load_map node of the st_map_server package.
 * @author Koen Lekkerkerker
 */

#include <st_map_server/save_map.h>

/*!
 * Main
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "st_save_map");
  ros::NodeHandle n;

  ROS_INFO("Hello World! Save Map");

  return 0;
}
