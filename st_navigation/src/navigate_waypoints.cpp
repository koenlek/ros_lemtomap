#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void multipleWayPoints1(move_base_msgs::MoveBaseGoal &goal, MoveBaseClient &ac);
void arg1WayPoint(move_base_msgs::MoveBaseGoal &goal, MoveBaseClient &ac, double x_goal, double y_goal,
                  double theta_goal);

int main(int argc, char** argv)
{
  // @TODO KL: also enable loading waypoints from a file (.yaml through param server or .txt plainly for example)
  ros::init(argc, argv, "navigate_waypoints");
  ros::NodeHandle private_nh_("~");

  // Parameters initialization
  std::string goal_ref_frame_id;
  private_nh_.param("goal_ref_frame_id", goal_ref_frame_id, std::string("map"));
  double x_goal;
  private_nh_.param("x_goal", x_goal, double(3.7));
  double y_goal;
  private_nh_.param("y_goal", y_goal, double(0.0));
  double theta_goal;
  private_nh_.param("theta_goal", theta_goal, double(0.01)); //KL: @TODO TODO: it seems like move_base goal cannot deal with theta=0?!

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = goal_ref_frame_id; // KL: use base_footprint to send relative goals, e.g. x=1 means 1 meter forward, or use /map to use world frame goals!
  goal.target_pose.header.stamp = ros::Time::now();

  //multipleWayPoints1(goal, ac);
  arg1WayPoint(goal, ac, x_goal,y_goal,theta_goal);

  return 0;
}

/**** KL: example of code that can send multiple waypoints.
 *
 *    @TODO TODO this piece of code is currently not very logically
 *    split from the rest: the ac and goal objects could also be completely part of it...
 *
 *    this function can also be used as an inline function (not tested),
 *    consequently it also be copy pasted in place in the main function. ******/
void multipleWayPoints1(move_base_msgs::MoveBaseGoal &goal, MoveBaseClient &ac)
{
  tf::TransformListener pose_listener;
  tf::StampedTransform pose_transform;

  const int n_wps = 4;
  double wps_matrix[n_wps][3] = { {4.4, 0.0, 1.00}, {7.4, -2.6, 0.90}, {2.7, -2.7, -0.24}, {10.2, 0.6, 0.96}};

  for (int i = 1; i <= n_wps; i++)
  {
    goal.target_pose.pose.position.x = wps_matrix[i - 1][0];
    goal.target_pose.pose.position.y = wps_matrix[i - 1][1];
    goal.target_pose.pose.orientation.w = wps_matrix[i - 1][2];

    ROS_INFO("Sending goal (x=%f, y=%f, theta=%f)", wps_matrix[i - 1][0], wps_matrix[i - 1][1], wps_matrix[i - 1][2]);
    ac.sendGoal(goal);

    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, sub-goal %i achieved", i);
      pose_listener.lookupTransform("/map", "/base_footprint", ros::Time(0), pose_transform);
      ROS_INFO("Robot pose is now: (x=%f, y=%f)", pose_transform.getOrigin().x(), pose_transform.getOrigin().y());
      ac.sendGoal(goal);
    }
    else
      ROS_INFO("Error, sub-goal %i failed", i);
  }
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot moved to all the required locations");
  else
    ROS_INFO("The robot failed to move to all the required locations for some reason");
}

/**** KL: example of code that processes 1 waypoint from node arguments.
 *
 *    @TODO TODO this piece of code is currently not very logically
 *    split from the rest: the ac and goal objects could also be completely part of it...
 *
 *    this function can also be used as an inline function (not tested),
 *    consequently it also be copy pasted in place in the main function. ******/
void arg1WayPoint(move_base_msgs::MoveBaseGoal &goal, MoveBaseClient &ac, double x_goal, double y_goal,
                  double theta_goal)
{
  tf::TransformListener pose_listener;
  tf::StampedTransform pose_transform;

  goal.target_pose.pose.position.x = x_goal;
  goal.target_pose.pose.position.y = y_goal;
  goal.target_pose.pose.orientation.w = theta_goal;

  ROS_INFO("Sending goal (x=%f, y=%f, theta=%f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
           goal.target_pose.pose.orientation.w);
  ac.sendGoal(goal);

  ac.waitForResult();
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, goal achieved");
    pose_listener.lookupTransform("/map", "/base_footprint", ros::Time(0), pose_transform);
    ROS_INFO("Robot pose is now: (x=%f, y=%f)", pose_transform.getOrigin().x(), pose_transform.getOrigin().y());
  }
  else
    ROS_INFO("Error, goal failed");
}
