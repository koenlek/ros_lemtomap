/**
 * @file publish_perfect_odom_frame.cpp
 * @author Koen Lekkerkerker
 */

#include <st_tools/publish_perfect_odom_frame.h>

PerfectOdomFrame::PerfectOdomFrame() {
	ros::NodeHandle private_nh("~");
	ros::NodeHandle n;

	private_nh.param("x_robot", x_init_, double(0.0));
	private_nh.param("y_robot", y_init_, double(0.0));

	get_gazebopose_servcli_ = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	geometry_msgs::Pose initial_pose ;
	bool received_valid = false;

	while(!received_valid && ros::ok()){
		initial_pose =  getTruePose();
		if (std::abs(initial_pose.position.x - x_init_) < 0.001 && std::abs(initial_pose.position.y - y_init_) < 0.001)
			received_valid = true;
		ros::Duration(0.5).sleep();
	}
}

void PerfectOdomFrame::updateFrame() {
	current_pose_ = getTruePose();
	//ROS_DEBUG("current pose position at (x,y)=(%.4f,%.4f)",current_pose_.position.x,current_pose_.position.y);
	/*ROS_DEBUG("current pose orientation at (x,y,z,w)=(%.4f,%.4f,%.4f,%.4f)",
			/current_pose_.orientation.x,
			current_pose_.orientation.y,
			current_pose_.orientation.z,
			current_pose_.orientation.w);*/

	tf::StampedTransform tf_odom_true;

	tf_odom_true.getOrigin().setX(current_pose_.position.x-x_init_);
	tf_odom_true.getOrigin().setY(current_pose_.position.y-y_init_);
	tf_odom_true.getOrigin().setZ(current_pose_.position.z);

	double theta = tf::getYaw(current_pose_.orientation);

	tf_odom_true.setRotation(tf::createQuaternionFromRPY(0,0,theta));

	br_.sendTransform(tf::StampedTransform(tf_odom_true.inverse(), ros::Time::now(), "base_footprint", "odom_ground_truth"));
}

geometry_msgs::Pose PerfectOdomFrame::getTruePose() {

	geometry_msgs::Pose true_pose;

	gazebo_msgs::GetModelState srv;
	srv.request.model_name = "mobile_base";

	if (!get_gazebopose_servcli_.call(srv)) {
		ROS_WARN("Did not receive a valid response from /gazebo/get_model_state service");
		return true_pose;
	}

	true_pose.position.x = srv.response.pose.position.x;
	true_pose.position.y = srv.response.pose.position.y;
	true_pose.position.z = srv.response.pose.position.z;
	true_pose.orientation.x = srv.response.pose.orientation.x;
	true_pose.orientation.y = srv.response.pose.orientation.y;
	true_pose.orientation.z = srv.response.pose.orientation.z;
	true_pose.orientation.w = srv.response.pose.orientation.w;

	ROS_DEBUG("Gazebo pose (x,y)= (%.4f,%.4f)",
			srv.response.pose.position.x,
			srv.response.pose.position.y);

	return true_pose;
}

/*!
 * Main
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, "perfect_odom_frame_broadcaster");
	ros::NodeHandle private_nh("~");
	ros::NodeHandle n;

	double frequency; //main loop frequency in Hz

	private_nh.param("main_loop_frequency", frequency, double(10.0));

	ros::Rate r(frequency);

	PerfectOdomFrame odom_broadcaster;

	while (n.ok()) {

		odom_broadcaster.updateFrame();
		ros::spinOnce(); // not sure if this is even needed:
		r.sleep();
		if (r.cycleTime() > ros::Duration(1 / frequency))
			ROS_WARN(
					"%s main loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds (%.4fHz)",
					ros::this_node::getName().c_str(), frequency,
					r.cycleTime().toSec(), 1 / r.cycleTime().toSec());
	}

	return 0;
}
