/**
 * @file odom_logger.cpp
 * @brief This file can log odom messages to a csv file.
 * @author Koen Lekkerkerker
 */

#include <lemto_tools/odom_logger.h>

OdomLogger::OdomLogger() {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle n;

  int queue_size;
  std::string topic;

  private_nh.param("store_path", store_path_, std::string("."));
  private_nh.param("queue_size", queue_size, int(1));
  private_nh.param("topic", topic, std::string("odom"));

  odom_sub_ = n.subscribe(topic, queue_size, &OdomLogger::odomCB, this);

  store_path_ = boost::filesystem::absolute(store_path_).c_str();

  std::string filename, full_path;
  filename = "odom_log";

  full_path = store_path_ + "/" + filename + ".csv";

  ROS_INFO("Start logging odom topic (%s) to:\n\t%s",topic.c_str(),full_path.c_str());

  logger_csv_ = fopen(full_path.c_str(), "w");
  if (!logger_csv_) {
    ROS_ERROR("Couldn't save odom topic data to:\n\t%s\nShutting down logger node now", full_path.c_str());
    n.shutdown();
  }
  fprintf(logger_csv_, "time, wall_time, x_pose, y_pose, twist_x, twist_theta, time_msg_processed, wall_time_msg_processed\n");

}

OdomLogger::~OdomLogger() {
  fclose (logger_csv_);
  ROS_INFO("Finished logging\n");
  std::cerr << "~OdomLogger: Finished logging odom topic" << std::endl;

}

void OdomLogger::odomCB(const nav_msgs::Odometry::ConstPtr &msg) {
  /**
   * There seems no way to convert ros (sim) time to wall time.
   * I think the best solution is to assume that the current time difference
   * between sim and wall time more or less equals the time difference at the
   * time at which the currently processed message was stamped...
   */
  double time_diff_now = ros::WallTime::now().toSec() - ros::Time::now().toSec();


  fprintf(logger_csv_, "%.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n",
          msg->header.stamp.toSec(),
          msg->header.stamp.toSec() + time_diff_now,
          msg->pose.pose.position.x,
          msg->pose.pose.position.y,
          msg->twist.twist.linear.x,
          msg->twist.twist.angular.z,
          ros::Time::now().toSec(),
          ros::WallTime::now().toSec());
}

/*!
 * Main
 */
int main(int argc, char** argv)
         {
ros::init(argc, argv, "odom_logger");
ros::NodeHandle private_nh("~");
ros::NodeHandle n;

double frequency; //main loop frequency in Hz

private_nh.param("main_loop_frequency", frequency, double(10.0));

ros::Rate r(frequency);

#ifdef CMAKE_BUILD_TYPE_DEF
#include <boost/preprocessor/stringize.hpp>
if (BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF) != "Release")
ROS_WARN("This node (%s) had CMAKE_BUILD_TYPE=%s. Please use catkin_make -DCMAKE_BUILD_TYPE=Release for benchmarks!", ros::this_node::getName().c_str(), BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF));
else
ROS_INFO("This node (%s) had CMAKE_BUILD_TYPE=%s.", ros::this_node::getName().c_str(), BOOST_PP_STRINGIZE(CMAKE_BUILD_TYPE_DEF));
#endif

OdomLogger logger;

while (n.ok()) {

ros::spinOnce();
r.sleep();
if (r.cycleTime() > ros::Duration(1 / frequency))
  ROS_WARN("%s main loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds (%.4fHz)", ros::this_node::getName().c_str(), frequency,
           r.cycleTime().toSec(),
           1 / r.cycleTime().toSec());
}

return 0;
}
