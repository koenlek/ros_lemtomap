#ifndef TIME_LOGGER_H
#define TIME_LOGGER_H

// General includes
#include "string"
#include <stdio.h>

// ROS includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <boost/filesystem.hpp>

/**
 * @file time_logger
 * @brief This file can log ros time and wall time to a csv file, to be able to convert wall time to ros time and vice versa when analyzing experiments later.
 * @author Koen Lekkerkerker
 */

class TimeLogger
{
private:
  /**
   * Variables
   */

  std::string store_path_;
  FILE* logger_csv_;

  /**
   *  Methods
   */

public:
  // Constructor/Destructor
  TimeLogger();
  ~TimeLogger();

  /**
   * Variables
   */

  /**
   * Methods
   */

  void updateLog();

};

#endif
