/**
 * @file time_logger.cpp
 * @brief This file can log ros time and wall time to a csv file, to be able to convert wall time to ros time and vice versa when analyzing experiments later.
 * @author Koen Lekkerkerker
 */

#include <lemto_tools/time_logger.h>

TimeLogger::TimeLogger() {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle n;

  int queue_size;
  std::string topic;

  private_nh.param("store_path", store_path_, std::string("."));

  store_path_ = boost::filesystem::absolute(store_path_).c_str();

  std::string filename, full_path;
  filename = "time_log";

  full_path = store_path_ + "/" + filename + ".csv";

  ROS_INFO("Start logging ROS and Wall time to:\n\t%s",full_path.c_str());

  logger_csv_ = fopen(full_path.c_str(), "w");
  if (!logger_csv_) {
    ROS_ERROR("Couldn't save time data to:\n\t%s\nShutting down logger node now", full_path.c_str());
    n.shutdown();
  }
  fprintf(logger_csv_, "ros_time, wall_time\n");
}

TimeLogger::~TimeLogger() {
  fclose (logger_csv_);
  ROS_INFO("Finished logging\n");
  std::cerr << "~TimeLogger: Finished logging time data" << std::endl;

}

void TimeLogger::updateLog() {
  fprintf(logger_csv_, "%.5f, %.5f\n",
          ros::Time::now().toSec(),
          ros::WallTime::now().toSec());
}

/*!
 * Main
 */
int main(int argc, char** argv)
         {
ros::init(argc, argv, "time_logger");
ros::NodeHandle private_nh("~");
ros::NodeHandle n;

double frequency; //main loop frequency in Hz

private_nh.param("main_loop_frequency", frequency, double(10.0));

ros::Rate r(frequency);

TimeLogger logger;

while (n.ok()) {

logger.updateLog();
ros::spinOnce(); // not sure if this is even needed:
r.sleep();
if (r.cycleTime() > ros::Duration(1 / frequency))
  ROS_WARN("%s main loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds (%.4fHz)", ros::this_node::getName().c_str(), frequency,
           r.cycleTime().toSec(),
           1 / r.cycleTime().toSec());
}

return 0;
}
