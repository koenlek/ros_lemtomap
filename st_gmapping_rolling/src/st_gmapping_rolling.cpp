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
/* Modified by: Charles DuHadway */
/* Further modified by: Koen Lekkerkerker */

/**

 @mainpage slam_gmapping

 @htmlinclude manifest.html

 @b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
 scans and odometry and computes a map. This map can be
 written to a file using e.g.

 "rosrun map_server map_saver static_map:=dynamic_map"

 <hr>

 @section topic ROS topics

 Subscribes to (name/type):
 - @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner
 - @b "/tf": odometry from the robot


 Publishes to (name/type):
 - @b "/tf"/tf/tfMessage: position relative to the map


 @section services
 - @b "~dynamic_map" : returns the map


 @section parameters ROS parameters

 Reads the following parameters from the parameter server

 Parameters used by our GMapping wrapper:

 - @b "~throttle_scans": @b [int] throw away every nth laser scan
 - @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
 - @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
 - @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
 - @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


 Parameters used by GMapping itself:

 Laser Parameters:
 - @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
 - @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
 - @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)
 - @b "~/kernelSize" @b [double] search window for the scan matching process
 - @b "~/lstep" @b [double] initial search step for scan matching (linear)
 - @b "~/astep" @b [double] initial search step for scan matching (angular)
 - @b "~/iterations" @b [double] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
 - @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
 - @b "~/ogain" @b [double] gain for smoothing the likelihood
 - @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)
 - @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

 Motion Model Parameters (all standard deviations of a gaussian noise model)
 - @b "~/srr" @b [double] linear noise component (x and y)
 - @b "~/stt" @b [double] angular noise component (theta)
 - @b "~/srt" @b [double] linear -> angular noise component
 - @b "~/str" @b [double] angular -> linear noise component

 Others:
 - @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
 - @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

 - @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
 - @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

 Likelihood sampling (used in scan matching)
 - @b "~/llsamplerange" @b [double] linear range
 - @b "~/lasamplerange" @b [double] linear step size
 - @b "~/llsamplestep" @b [double] linear range
 - @b "~/lasamplestep" @b [double] angular setp size

 Initial map dimensions and resolution:
 - @b "~/xmin" @b [double] minimum x position in the map [m]
 - @b "~/ymin" @b [double] minimum y position in the map [m]
 - @b "~/xmax" @b [double] maximum x position in the map [m]
 - @b "~/ymax" @b [double] maximum y position in the map [m]
 - @b "~/delta" @b [double] size of one pixel [m]

 Rolling window:
 - @b "~/windowsize" @b [double] size of the rolling window [m] (will overrule xmin, ymin, xmax, and ymax)

 Extra:
 - @b "~/publishCurrentPath" @b [bool] Publish RVIZ visualizable path for current particle. (default: false)
 - @b "~/publishAllPaths" @b [bool] Publish RVIZ visualizable paths for each particle. WARNING: can be cpu intensive! (default: false)
 - @b "~/publishSpecificMap" @b [int]  Publish RVIZ visualizable maps for a specific particle, as originally stored in that particle. (for debugging facilities) WARNING: can be cpu intensive! (default: -1, means disabled)

 */

#include "st_gmapping_rolling.h"

#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include "gmapping/sensor/sensor_range/rangesensor.h" //KL: these are openslam_gmapping headers -> openslam_gmapping hasnt used the pkg/include/pkgname/headers (e.g.  openslam_gmapping/include/openslam_gmapping/header.h) structure. it is: openslam_gmapping/include/gmapping/header.h instead, which becomes just this in install space: include/gmapping/header.h
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping() :
    map_to_odom_(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Point(0, 0, 0))),
    laser_count_(0), transform_thread_(NULL)
{
  // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  // The library is pretty chatty
  //gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  ROS_ASSERT(gsp_);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  gsp_laser_ = NULL;
  gsp_laser_angle_increment_ = 0.0;
  gsp_odom_ = NULL;

  got_first_scan_ = false;
  got_map_ = false;

  ros::NodeHandle private_nh_("~");

  // Parameters used by our GMapping wrapper
  if (!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if (!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if (!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  double transform_publish_period;
  private_nh_.param("transform_publish_period", transform_publish_period, 0.05);

  double tmp;
  if (!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);

  // Parameters used by GMapping itself
  maxUrange_ = 0.0;
  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
  if (!private_nh_.getParam("sigma", sigma_))
    sigma_ = 0.05;
  if (!private_nh_.getParam("kernelSize", kernelSize_))
    kernelSize_ = 1;
  if (!private_nh_.getParam("lstep", lstep_))
    lstep_ = 0.05;
  if (!private_nh_.getParam("astep", astep_))
    astep_ = 0.05;
  if (!private_nh_.getParam("iterations", iterations_))
    iterations_ = 5;
  if (!private_nh_.getParam("lsigma", lsigma_))
    lsigma_ = 0.075;
  if (!private_nh_.getParam("ogain", ogain_))
    ogain_ = 3.0;
  if (!private_nh_.getParam("lskip", lskip_))
    lskip_ = 0;
  if (!private_nh_.getParam("srr", srr_))
    srr_ = 0.1;
  if (!private_nh_.getParam("srt", srt_))
    srt_ = 0.2;
  if (!private_nh_.getParam("str", str_))
    str_ = 0.1;
  if (!private_nh_.getParam("stt", stt_))
    stt_ = 0.2;
  if (!private_nh_.getParam("linearUpdate", linearUpdate_))
    linearUpdate_ = 1.0;
  if (!private_nh_.getParam("angularUpdate", angularUpdate_))
    angularUpdate_ = 0.5;
  if (!private_nh_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  if (!private_nh_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  if (!private_nh_.getParam("particles", particles_))
    particles_ = 30;
  if (!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;
  if (!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;
  if (!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;
  if (!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;
  if (!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  if (!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  if (!private_nh_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  if (!private_nh_.getParam("llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;
  if (!private_nh_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  if (!private_nh_.getParam("lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;
  if (!private_nh_.getParam("minimumScore", minimum_score_))
    minimum_score_ = 0;
  if (!private_nh_.getParam("windowsize", windowsize_)) {
    windowsize_ = 0;
    rolling_ = false;
  }
  else {
    rolling_ = true;
  }
  if(!private_nh_.getParam("rolling_window_option", rolling_window_option_)){
    rolling_window_option_ = 0; //KL TMP
  private_nh_.setParam("rolling_window_option", rolling_window_option_);
  }

  if (!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period;

  //KL Visualize and store all paths / maps
  if (!private_nh_.getParam("publishAllPaths", publish_all_paths_))
    publish_all_paths_ = false;
  if (!private_nh_.getParam("publishCurrentPath", publish_current_path_))
    publish_current_path_ = false;

  if (!private_nh_.getParam("publishSpecificMap", publish_specific_map_))
    publish_specific_map_ = -1;
  if (publish_specific_map_ > particles_ || publish_specific_map_ < -1) {
    ROS_ERROR("publishSpecificMap for particle %d impossible, as total particles are %d, so value should be 0 to %d for specific particle, or %d for best particle. Publishing specific map is disabled now.", publish_specific_map_, particles_, particles_ - 1,particles_);
    publish_specific_map_ = -1;
  }
  else if (publish_specific_map_ == particles_) {
    ROS_INFO("Will publish specific smap for best particle");
  }
  else{
    ROS_INFO("Will publish specific smap for particle %d", publish_specific_map_);
  }

  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

  // KL Visualize and store all paths / maps
  if (publish_all_paths_) {
    paths_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>("all_paths", 1, true);
    path_m_.header.frame_id = tf_.resolve(map_frame_);
    path_m_.header.stamp = ros::Time();
    path_m_.action = visualization_msgs::Marker::ADD;
    path_m_.type = visualization_msgs::Marker::LINE_STRIP;
    path_m_.scale.x = 0.02;
    path_m_.color.r = 0.1;
    path_m_.color.g = 0.1;
    path_m_.color.b = 0.1;
    path_m_.color.a = 0.5;
  }
  if (publish_current_path_) {
    current_path_publisher_ = private_nh_.advertise<nav_msgs::Path>("current_path", 1, true);
  }
  if (publish_current_path_ || publish_all_paths_) {
    all_paths_.resize(particles_);
    for (int i = 0; i < all_paths_.size(); i++) {
      all_paths_.at(i).header.frame_id = tf_.resolve(map_frame_);
    }
  }
  if (publish_specific_map_ >= 0) {
    got_map_px_ = false;
    map_px_publisher_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("map_px", 1, true);
    map_px_info_publisher_ = private_nh_.advertise<nav_msgs::MapMetaData>("map_px_metadata", 1, true);
  }
#if DEBUG
  tests_performed_ = 0;
#endif

  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));

  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period));
}

void SlamGMapping::publishLoop(double transform_publish_period)
{
  if (transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while (ros::ok()) {
    publishTransform();
    r.sleep();
  }
}

SlamGMapping::~SlamGMapping()
{
  if (transform_thread_) {
    transform_thread_->join();
    delete transform_thread_;
  }

  delete gsp_;
  if (gsp_laser_)
    delete gsp_laser_;
  if (gsp_odom_)
    delete gsp_odom_;
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

bool
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  // Get the laser's pose
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),
                                            tf::Vector3(0, 0, 0)),
                              t, laser_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch (tf::TransformException &e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

bool
SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;
  try
  {
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  catch (tf::TransformException &e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  //KL: up is used for some tricks: including checking if laser is upside down (and auto correct it) and to check if the scans are completely planar (zero roll/pitch!)
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                              base_frame_);
  try
  {
    tf_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }

  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
      {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
             up.z());
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();

  int orientationFactor;
  if (up.z() > 0)
      {
    orientationFactor = 1;
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    orientationFactor = -1;
    ROS_INFO("Laser is mounted upside down.");
  }

  angle_min_ = orientationFactor * scan.angle_min;
  angle_max_ = orientationFactor * scan.angle_max;
  gsp_laser_angle_increment_ = orientationFactor * scan.angle_increment;
  ROS_DEBUG("Laser angles top down in laser-frame: min: %.3f max: %.3f inc: %.3f", angle_min_, angle_max_, gsp_laser_angle_increment_);

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~");
  if (!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if (!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,
                                         fabs(gsp_laser_angle_increment_),
                                         gmap_pose,
                                         0.0,
                                         maxRange_);
  ROS_ASSERT(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);

  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  if (!getOdomPose(initialPose, scan.header.stamp))
                   {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }

  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_,
                              lstep_, astep_, iterations_,
                              lsigma_,
                              ogain_, lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
//  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_,
                                initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1, time(NULL));

  ROS_INFO("Initialization complete");

  return true;
}

bool SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  if (!getOdomPose(gmap_pose, scan.header.stamp))
    return false;

  if (scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (gsp_laser_angle_increment_ < 0)
      {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan.ranges.size();
    for (int i = 0; i < num_ranges; i++)
        {
      // Must filter out short readings, because the mapper won't
      if (scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  }
  else
  {
    for (unsigned int i = 0; i < scan.ranges.size(); i++)
        {
      // Must filter out short readings, because the mapper won't
      if (scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);

  /*
   ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
   scan.header.stamp.toSec(),
   gmap_pose.x,
   gmap_pose.y,
   gmap_pose.theta);
   */

  return gsp_->processScan(reading);
}

void
SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0, 0);

  // We can't initialize the mapper until we've got the first scan
  if (!got_first_scan_)
  {
    if (!initMapper(*scan)) //
      return;
    got_first_scan_ = true;
    //ROS_INFO("best particle weight:%.4f",gsp_->getParticles()[gsp_->getBestParticleIndex()].weight); //TMP
    //ROS_INFO("total particle weight:%.4f",gsp_->getParticles()[gsp_->getBestParticleIndex()].weightSum);  //TMP
  }

  GMapping::OrientedPoint odom_pose;
  if (addScan(*scan, odom_pose))
              {
    ROS_DEBUG("scan processed");

    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    if (!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
        {
      updateMap(*scan);
      last_map_update = scan->header.stamp;
      ROS_DEBUG("Updated the map");
    }
  }
}

double
SlamGMapping::computePoseEntropy()
{
  double weight_total = 0.0;
  for (std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
      {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for (std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
      {
    if (it->weight / weight_total > 0.0)
      entropy += it->weight / weight_total * log(it->weight / weight_total);
  }
  return -entropy;
}

//KL: udpateMap is only run every map_update_interval_ seconds.
void SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
  boost::mutex::scoped_lock map_lock(map_mutex_);
  GMapping::ScanMatcher matcher;
  double* laser_angles = new double[scan.ranges.size()];
  double theta = angle_min_;
  for (unsigned int i = 0; i < scan.ranges.size(); i++)
      {
    if (gsp_laser_angle_increment_ < 0)
      laser_angles[scan.ranges.size() - i - 1] = theta;
    else
      laser_angles[i] = theta;
    theta += gsp_laser_angle_increment_;
  }

  matcher.setLaserParameters(scan.ranges.size(), laser_angles,
                             gsp_laser_->getPose());

  delete[] laser_angles;
  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
      gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if (entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

  if (!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  GMapping::Point center;
  center.x = (xmin_ + xmax_) / 2.0;
  center.y = (ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, delta_);
  //ROS_INFO("before smap2 copy"); //KL TMP
  //GMapping::ScanMatcherMap smap2 = best.map; //KL TMP
  //ROS_INFO("after smap2 copy"); //KL TMP

  ROS_DEBUG("Trajectory tree:");
  for (GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
          {
    ROS_DEBUG("  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if (!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    /* This works kind of, but is not right yet. It properly seems to delete links to previous measurements (not measurements itself!). also, it needs measurements from outside window to mark parts inside window as free. So not working right!*/
    /*if (rolling_window_option_ == 1){
      if ( n->pose.x < xmin_ || n->pose.x > xmax_ || n->pose.y < ymin_ || n->pose.y > ymax_){
       ROS_DEBUG("TNode is out of area, measurement is set to 0");
       n->reading->clear(); //KL: first had to make it r/w accessible, but now I can clear rangereading objects (vectors)
       continue;
      }
    }*/
    //KL TMP
    //ROS_INFO("before smap2 matcher steps");
    //matcher.invalidateActiveArea();
    //matcher.computeActiveArea(smap2, n->pose, &((*n->reading)[0]));
    //matcher.registerScan(smap2, n->pose, &((*n->reading)[0]));
    //ROS_INFO("after smap2 matcher steps");
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  // if the map has expanded, resize the map msg and all particle GMapping::ScanMatcherMaps
  if (map_.map.info.width != (unsigned int)smap.getMapSizeX() || map_.map.info.height != (unsigned int)smap.getMapSizeY()) {
    if (rolling_) {
      tf::StampedTransform robot_pose;
      try
      {
        //tf_.waitForTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
        tf_.lookupTransform(map_frame_, base_frame_, ros::Time(0),
                            robot_pose);
      }
      catch (tf::TransformException &e)
      {
        ROS_WARN("Failed to compute robot pose (%s)", e.what());
      }
      ROS_DEBUG("Resizing map for rolling window gmapping: windowsize = %.4f[m]", windowsize_);
      ROS_DEBUG("map_.map.info.origin.position (x,y) = (%.4f, %.4f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
      ROS_DEBUG("map_.map.info.resolution = %.4f\n", map_.map.info.resolution);
      ROS_DEBUG("robot pose (%.4f , %.4f)", robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY());

      ROS_DEBUG("Before resize smap: xmin %.4f, ymin %.4f, xmax %.4f, ymax %.4f, center (%.4f , %.4f)", xmin_, ymin_, xmax_, ymax_, (xmin_ + xmax_) / 2.0, (ymin_ + ymax_) / 2.0);
      ROS_DEBUG("smap size x = %d", smap.getMapSizeX());
      ROS_DEBUG("smap size y = %d", smap.getMapSizeY());
      ROS_DEBUG("smap world size x = %.4f", smap.getWorldSizeX());
      ROS_DEBUG("smap world size y = %.4f", smap.getWorldSizeY());

      xmin_ = -windowsize_ / 2 + robot_pose.getOrigin().getX();
      ymin_ = -windowsize_ / 2 + robot_pose.getOrigin().getY();
      xmax_ = windowsize_ / 2 + robot_pose.getOrigin().getX();
      ymax_ = windowsize_ / 2 + robot_pose.getOrigin().getY();

      //update the map used for visualization
      smap.resize(xmin_, ymin_, xmax_, ymax_);
      //update all the maps stored in the particles
      /*for (int i = 0; i < particles_; i++) {
        gsp_->getParticlesRW().at(i).map.resize(xmin_, ymin_, xmax_, ymax_);
      }*/

      ROS_DEBUG("After resize smap: xmin %.4f, ymin %.4f, xmax %.4f, ymax %.4f, center (%.4f , %.4f)", xmin_, ymin_, xmax_, ymax_, (xmin_ + xmax_) / 2.0, (ymin_ + ymax_) / 2.0);
      ROS_DEBUG("smap size x = %d", smap.getMapSizeX());
      ROS_DEBUG("smap size y = %d", smap.getMapSizeY());
      ROS_DEBUG("smap world size x = %.4f", smap.getWorldSizeX());
      ROS_DEBUG("smap world size y = %.4f\n", smap.getWorldSizeY());
    }

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x;
    ymin_ = wmin.y; //KL: this is where the map size is updated if it is growing
    xmax_ = wmax.x;
    ymax_ = wmax.y;

    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for (int x = 0; x < smap.getMapSizeX(); x++)
      {
    for (int y = 0; y < smap.getMapSizeY(); y++)
        {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ = smap.cell(p);
      assert(occ <= 1.0);
      if (occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if (occ > occ_thresh_)
          {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve(map_frame_);

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);

  // KL Visualize and store all paths / maps
  ROS_INFO("Best particle is %d", gsp_->getBestParticleIndex());
  if (publish_all_paths_ || publish_current_path_) {
    updateAllPaths();
    if (publish_all_paths_) {
      publishAllPaths();
    }
    if (publish_current_path_) {
      publishCurrentPath();
    }
  }
  if (publish_specific_map_ >= 0) {
    publishMapPX();
  }

#if DEBUG
  if (ros::Time::now() > ros::Time(100) && tests_performed_ < 1) {
    tests_performed_++;
    smapToCSV(smap, "smap_forvismap"); //currently visualized smap
    //smapToCSV(smap2,"smap2_forvismap"); //currently visualized smap
    smapToCSV(gsp_->getParticles()[gsp_->getBestParticleIndex()].map, "smap_frombestparticle"); //any smap
  }
#endif

}

//KL Visualize and store all paths / maps
geometry_msgs::Pose SlamGMapping::gMapPoseToGeoPose(const GMapping::OrientedPoint& gmap_pose) const
{
  geometry_msgs::Pose geo_pose;

  geo_pose.position.x = gmap_pose.x;
  geo_pose.position.y = gmap_pose.y;
  geo_pose.orientation = tf::createQuaternionMsgFromYaw(gmap_pose.theta);

  return geo_pose;
}

//KL Visualize and store all paths / maps
void SlamGMapping::updateAllPaths()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = tf_.resolve(map_frame_);

  //TODO - p2 - This recreates all the paths every time, which is causing extra system load. Maybe only update it (add latest poses). Make sure that it survives resampling properly though!!!

  for (int i = 0; i < particles_; i++) {
    all_paths_.at(i).poses.clear();

    //t_node_current_ = gsp_->getTrajectories().at(i); //KL: do not use getTrajectories -> causes some weird memory leak that I do not understand.
    t_node_current_ = gsp_->getParticles().at(i).node;

    while (t_node_current_ != 0) {
      //ROS_INFO("t_node_current->pose (x,y,theta) = (%.4f,%.4f,%.4f)",t_node_current_->pose.x,t_node_current_->pose.y,t_node_current_->pose.theta);
      pose.pose = gMapPoseToGeoPose(t_node_current_->pose);
      all_paths_.at(i).poses.push_back(pose);
      t_node_current_ = t_node_current_->parent;
    }
  }
}

//KL Visualize and store all paths / maps
void SlamGMapping::publishMapPX()
{
  if (publish_specific_map_ == particles_){
    publish_specific_map_ = gsp_->getBestParticleIndex();
  }
  const GMapping::GridSlamProcessor::Particle &current_p = gsp_->getParticles()[publish_specific_map_];

  if (!got_map_px_) {
    map_px_.map.info.resolution = delta_;
    map_px_.map.info.origin.position.x = 0.0;
    map_px_.map.info.origin.position.y = 0.0;
    map_px_.map.info.origin.position.z = 0.0;
    map_px_.map.info.origin.orientation.x = 0.0;
    map_px_.map.info.origin.orientation.y = 0.0;
    map_px_.map.info.origin.orientation.z = 0.0;
    map_px_.map.info.origin.orientation.w = 1.0;
  }

  //ROS_INFO("current_p.map.getMapSizeX() = (x,y) (%d, %d)",current_p.map.getMapSizeX(),current_p.map.getMapSizeY());
  //ROS_INFO("current_p.map.pose (x,y,theta) = (%.4f, %.4f, %.4f)",current_p.pose.x,current_p.pose.y,current_p.pose.theta);

  //set the center of the map
  double xmin_px, xmax_px, ymin_px, ymax_px;

  //resize map if needed
  if (map_px_.map.info.width != (unsigned int)current_p.map.getMapSizeX() || map_px_.map.info.height != (unsigned int)current_p.map.getMapSizeY()) {
    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = current_p.map.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = current_p.map.map2world(GMapping::IntPoint(current_p.map.getMapSizeX(), current_p.map.getMapSizeY()));
    xmin_px = wmin.x;
    ymin_px = wmin.y; //KL: this is where the map size is updated if it is growing
    xmax_px = wmax.x;
    ymax_px = wmax.y;
    map_px_.map.info.width = current_p.map.getMapSizeX();
    map_px_.map.info.height = current_p.map.getMapSizeY();
    map_px_.map.info.origin.position.x = xmin_px;
    map_px_.map.info.origin.position.y = ymin_px;

    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", current_p.map.getMapSizeX(), current_p.map.getMapSizeY(),
              xmin_px, ymin_px, xmax_px, ymax_px);
    ROS_DEBUG("wmin.x = %.3f, wmin.y = %.3f, wmax.x = %.3f, wmax.y = %.3f", wmin.x, wmin.y, wmax.x, wmax.y);

    map_px_.map.data.resize(map_px_.map.info.width * map_px_.map.info.height);
    ROS_DEBUG("map origin: (%f, %f)", map_px_.map.info.origin.position.x, map_px_.map.info.origin.position.y);
  }

  for (int x = 0; x < current_p.map.getMapSizeX(); x++)
      {
    for (int y = 0; y < current_p.map.getMapSizeY(); y++)
        {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ = current_p.map.cell(p);
      assert(occ <= 1.0);
      if (occ < 0)
        map_px_.map.data[MAP_IDX(map_px_.map.info.width, x, y)] = -1;
      else if (occ > occ_thresh_)
          {
        //map_px_.map.data[MAP_IDX(map_px_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_px_.map.data[MAP_IDX(map_px_.map.info.width, x, y)] = 100;
      }
      else
        map_px_.map.data[MAP_IDX(map_px_.map.info.width, x, y)] = 0;
    }
  }
  got_map_px_ = true;

  //make sure to set the header information on the map
  map_px_.map.header.stamp = ros::Time::now();
  map_px_.map.header.frame_id = tf_.resolve(map_frame_);

  map_px_publisher_.publish(map_px_.map);
  map_px_info_publisher_.publish(map_px_.map.info);

}

//KL Visualize and store all paths / maps
void SlamGMapping::publishCurrentPath()
{
  current_path_publisher_.publish(all_paths_.at(gsp_->getBestParticleIndex()));
}

//KL Visualize and store all paths / maps
void SlamGMapping::publishAllPaths()
{
  path_m_.header.stamp = ros::Time::now();
  all_paths_ma_.markers.clear();

  for (int i_particle = 0; i_particle < particles_; i_particle++) {
    path_m_.points.clear();
    path_m_.id = i_particle;
    std::ostringstream stringStream;
    stringStream << "path_particle" << i_particle;
    path_m_.ns = stringStream.str();
    for (int i = 0; i < all_paths_.at(i_particle).poses.size(); i++) {
      path_m_.points.push_back(all_paths_.at(i_particle).poses.at(i).pose.position);
    }
    all_paths_ma_.markers.push_back(path_m_);
  }
  paths_publisher_.publish(all_paths_ma_);
}

bool SlamGMapping::mapCallback(nav_msgs::GetMap::Request &req,
                               nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock map_lock(map_mutex_);
  if (got_map_ && map_.map.info.width && map_.map.info.height)
      {
    res = map_;
    return true;
  }
  else
    return false;
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tfB_->sendTransform(tf::StampedTransform(map_to_odom_, tf_expiration, map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}

#if DEBUG
void SlamGMapping::smapToCSV(GMapping::ScanMatcherMap smap, std::string filename)
{
  filename = filename + ".csv";
  FILE* smap_csv = fopen(filename.c_str(), "w");
  if (!smap_csv) {
    ROS_ERROR("Couldn't save map file to %s.csv",
              filename.c_str());
    return;
  }
  int sizex = smap.getMapSizeX();
  int sizey = smap.getMapSizeY();

  for (int y = 0; y < sizey; y++) {
    for (int x = 0; x < sizex; x++) {
      GMapping::IntPoint p(x, y);
      double occ = smap.cell(p);
      //fprintf(smap_csv, "(x;y)(%d;%d)",x,y); //uncomment to check x,y
      fprintf(smap_csv, "%.4f", occ);
      if (x != sizex - 1)
        fprintf(smap_csv, ",");
    }
    fprintf(smap_csv, "\n");
  }

  fclose(smap_csv);

  ROS_INFO("Done saving\n");
}
#endif
