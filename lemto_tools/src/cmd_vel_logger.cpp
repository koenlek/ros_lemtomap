/**
 * @file cmd_vel_logger.cpp
 * @brief This file can log velocity command messages (geometry_msgs/Twist) to a csv file.
 * @author Koen Lekkerkerker
 */

#include <lemto_tools/cmd_vel_logger.h>

CmdVelLogger::CmdVelLogger() {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle n;

  int queue_size;
  std::string topic;

  private_nh.param("store_path", store_path_, std::string("."));
  private_nh.param("queue_size", queue_size, int(1));
  private_nh.param("topic", topic, std::string("mobile_base/commands/velocity"));

  cmd_vel_sub_ = n.subscribe(topic, queue_size, &CmdVelLogger::cmdVelCB, this);

  store_path_ = boost::filesystem::absolute(store_path_).c_str();

  std::string filename, full_path;
  filename = "cmd_vel_log";

  full_path = store_path_ + "/" + filename + ".csv";

  ROS_INFO("Start logging cmd vel topic (%s) to:\n\t%s",topic.c_str(),full_path.c_str());

  logger_csv_ = fopen(full_path.c_str(), "w");
  if (!logger_csv_) {
    ROS_ERROR("Couldn't save cmd vel topic data to:\n\t%s\nShutting down logger node now", full_path.c_str());
    n.shutdown();
  }
  fprintf(logger_csv_, "time_msg_processed, wall_time_msg_processed, twist_x, twist_theta\n");

}

CmdVelLogger::~CmdVelLogger() {
  fclose (logger_csv_);
  ROS_INFO("Finished logging\n");
  std::cerr << "~CmdVelLogger: Finished logging cmd vel topic" << std::endl;

}

void CmdVelLogger::cmdVelCB(const geometry_msgs::Twist::ConstPtr &msg) {

  fprintf(logger_csv_, "%.5f, %.5f, %.5f, %.5f\n",
          ros::Time::now().toSec(),
          ros::WallTime::now().toSec(),
          msg->linear.x,
          msg->angular.z);
}

/*!
 * Main
 */
int main(int argc, char** argv)
         {
ros::init(argc, argv, "cmd_vel_logger");
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

CmdVelLogger logger;

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
