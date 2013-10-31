// Subscribes to state, pose
// Has hard coded trajectory (along x axis - [0, 0, 0], [1, 0, 0], etc)
// Publishes state, pose, path

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <vehicle_description/State.h>

#define PI 3.141

geometry_msgs::Pose pose;
vehicle_description::State state;
double error;

void poseUpdate(const geometry_msgs::Pose::ConstPtr& _pose) {
	pose.position = _pose->position;
	pose.orientation = _pose->orientation;
	
	double roll, pitch, yaw;
	tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	tf::Matrix3x3 matrix(quat);
	matrix.getRPY(roll, pitch, yaw);
	
	ROS_INFO("True Pose2D: [%.2lf, %.2lf, %.2lf]", pose.position.x, pose.position.y, yaw * 180 / PI);
	error = (error + sqrt(((pose.position.x - pose.position.y) * (pose.position.x - pose.position.y)) / 2)) / 2;
	ROS_INFO("TrajFollow Error: [%.2lf]", error);
}

void stateUpdate(const vehicle_description::State::ConstPtr& _state) {
	state.rear_wheel_speed = _state->rear_wheel_speed;
	state.steer_angle = _state->steer_angle;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	// Observed stuff from simulator. Noise can be added later.
	ros::Subscriber state_sub = n.subscribe("simulator/state", 2, stateUpdate);
	ros::Subscriber pose_sub = n.subscribe("simulator/pose", 2, poseUpdate);
	
	// Mimics other nodes
	ros::Publisher state_pub = n.advertise<vehicle_description::State>("vehicle_server/state", 20);
	ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("localization/pose", 100);
	ros::Publisher path_pub = n.advertise<nav_msgs::Path>("local_planner/path", 10);
	
	int n_poses = 100;
	nav_msgs::Path path_msg;
	path_msg.poses.resize(n_poses);
	for (int i = 0; i < path_msg.poses.size(); i++) {
		path_msg.poses[i].pose.position.x = i;
		path_msg.poses[i].pose.position.y = i;
		path_msg.poses[i].pose.position.z = 0;	
		path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -PI / 4);
	}
	
	ros::Rate loop_rate(100);
	int count = 0;
	error = 0;
	while (ros::ok()) {
		state_pub.publish(state);
		pose_pub.publish(pose);
		
		path_msg.header.seq = count;
		path_msg.header.frame_id = count;
		path_msg.header.stamp = ros::Time::now();
		path_pub.publish(path_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}
