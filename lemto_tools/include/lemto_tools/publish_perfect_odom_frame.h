#ifndef PUBLISH_PERFECT_ODOM_FRAME_H
#define PUBLISH_PERFECT_ODOM_FRAME_H

// General includes
#include "string"
#include <stdio.h>

// ROS includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <boost/filesystem.hpp>

#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"

#include "gazebo_msgs/GetModelState.h"

/**
 * @file publish_perfect_odom_frame
 * @author Koen Lekkerkerker
 */

class PerfectOdomFrame {
private:
	/**
	 * Variables
	 */

	ros::ServiceClient get_gazebopose_servcli_;
	double x_init_, y_init_;
	geometry_msgs::Pose current_pose_;
    tf::TransformBroadcaster br_;

	/**
	 *  Methods
	 */
	geometry_msgs::Pose getTruePose();

public:
	// Constructor/Destructor
	PerfectOdomFrame();

	/**
	 * Variables
	 */

	/**
	 * Methods
	 */

	void updateFrame();

};

#endif
