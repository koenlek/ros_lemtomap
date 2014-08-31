#ifndef ODOM_LOGGER_H
#define ODOM_LOGGER_H

// General includes
#include "string"
#include <stdio.h>

// ROS includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <boost/filesystem.hpp>

/**
 * @file odom_logger
 * @brief This file can log odom messages to a csv file.
 * @author Koen Lekkerkerker
 */

class OdomLogger
{
private:
  /**
   * Variables
   */

  std::string store_path_;
  ros::Subscriber odom_sub_;
  FILE* logger_csv_;

  /**
   *  Methods
   */

public:
  // Constructor/Destructor
  OdomLogger();
  ~OdomLogger();

  /**
   * Variables
   */

  /**
   * Methods
   */

  void odomCB(const nav_msgs::Odometry::ConstPtr &msg);

};

#endif
