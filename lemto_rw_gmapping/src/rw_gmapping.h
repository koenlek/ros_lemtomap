/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

#include <boost/thread.hpp>

// KL Visualize and store all paths / maps
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include "tf/transform_datatypes.h"
#include <stdio.h>

class SlamGMappingRolling
{
public:
  SlamGMappingRolling();
  ~SlamGMappingRolling();

  void publishTransform();

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool mapCallback(nav_msgs::GetMap::Request &req,
                   nav_msgs::GetMap::Response &res);
  void publishLoop(double transform_publish_period);

private:

  ros::NodeHandle node_;
  ros::Publisher entropy_publisher_;
  ros::Publisher sst_;
  ros::Publisher sstm_;
  ros::ServiceServer ss_;
  tf::TransformListener tf_;
  message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
  tf::TransformBroadcaster* tfB_;

  GMapping::GridSlamProcessor* gsp_;
  GMapping::RangeSensor* gsp_laser_;
  double gsp_laser_angle_increment_;
  double angle_min_;
  double angle_max_;
  unsigned int gsp_laser_beam_count_;
  GMapping::OdometrySensor* gsp_odom_;

  bool got_first_scan_;

  bool got_map_;
  nav_msgs::GetMap::Response map_;

  ros::Duration map_update_interval_;
  tf::Transform map_to_odom_;
  boost::mutex map_to_odom_mutex_;
  boost::mutex map_mutex_;

  int laser_count_;
  int throttle_scans_;

  boost::thread* transform_thread_;

  std::string base_frame_;
  std::string laser_frame_;
  std::string map_frame_;
  std::string odom_frame_;

  void updateMap(const sensor_msgs::LaserScan& scan);
  bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);
  bool initMapper(const sensor_msgs::LaserScan& scan);
  bool addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose);
  double computePoseEntropy();

  // Parameters used by GMapping
  double maxRange_;
  double maxUrange_;
  double maxrange_;
  double minimum_score_;
  double sigma_;
  int kernelSize_;
  double lstep_;
  double astep_;
  int iterations_;
  double lsigma_;
  double ogain_;
  int lskip_;
  double srr_; //Odometry error in translation as a function of translation (rho/rho)
  double srt_; //Odometry error in translation as a function of rotation (rho/theta)
  double str_; //Odometry error in rotation as a function of translation (theta/rho)
  double stt_; //Odometry error in rotation as a function of rotation (theta/theta)
  double linearUpdate_;
  double angularUpdate_;
  double temporalUpdate_;
  double resampleThreshold_;
  int particles_;
  double xmin_;
  double ymin_;
  double xmax_;
  double ymax_;
  double delta_;
  double occ_thresh_;
  double llsamplerange_;
  double llsamplestep_;
  double lasamplerange_;
  double lasamplestep_;

  double tf_delay_;

  double windowsize_;
  bool rolling_; //KL

  // Visualize and store all paths and maps
  geometry_msgs::Pose gMapPoseToGeoPose(const GMapping::OrientedPoint& gmap_pose) const;
  void updateAllPaths();
  void publishCurrentPath();
  void publishAllPaths();
  void publishMapPX();
  std::vector<nav_msgs::Path> all_paths_;
  visualization_msgs::MarkerArray all_paths_ma_;
  visualization_msgs::Marker path_m_;
  ros::Publisher paths_publisher_; //KL
  ros::Publisher current_path_publisher_; //KL
  GMapping::GridSlamProcessor::TNode* t_node_current_;
  bool publish_all_paths_;
  bool publish_current_path_;
  int publish_specific_map_; //int stands for particle index
  bool visualize_robot_centric_;

  //map publishing
  ros::Publisher map_px_publisher_; //KL
  ros::Publisher map_px_info_publisher_; //KL
  nav_msgs::GetMap::Response map_px_;
  bool got_map_px_;

  std::vector<GMapping::ScanMatcherMap> smap_vector_; //for resize
  void updateMapDefault(const sensor_msgs::LaserScan& scan, GMapping::ScanMatcherMap& smap);
  void updateMapRollingMode(const sensor_msgs::LaserScan& scan, GMapping::ScanMatcherMap& smap, bool& scan_out_of_smap);
  void updateMapOrig(const sensor_msgs::LaserScan& scan);

  void resizeMapMsg(const GMapping::ScanMatcherMap &smap);
  void resizeAllSMaps(GMapping::ScanMatcherMap &smap, bool including_particles = true);

};
