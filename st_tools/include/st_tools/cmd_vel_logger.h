#ifndef CMD_VEL_LOGGER_H
#define CMD_VEL_LOGGER_H

// General includes
#include "string"
#include <stdio.h>

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <boost/filesystem.hpp>

/**
 * @file cmd_vel_logger
 * @brief This file can log velocity command messages (geometry_msgs/Twist) to a csv file.
 * @author Koen Lekkerkerker
 */

class CmdVelLogger
{
private:
  /**
   * Variables
   */

  std::string store_path_;
  ros::Subscriber cmd_vel_sub_;
  FILE* logger_csv_;

  /**
   *  Methods
   */

public:
  // Constructor/Destructor
  CmdVelLogger();
  ~CmdVelLogger();

  /**
   * Variables
   */

  /**
   * Methods
   */

  void cmdVelCB(const geometry_msgs::Twist::ConstPtr &msg);

};

#endif
