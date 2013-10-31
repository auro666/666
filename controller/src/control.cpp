#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <vehicle_description/State.h>

#define PI 3.141

int path_ended = 0;

geometry_msgs::Pose pose;
vehicle_description::State state;
nav_msgs::Path path;

void poseUpdate(const geometry_msgs::Pose::ConstPtr& _pose) {
	pose.position = _pose->position;
	pose.orientation = _pose->orientation;
}

void stateUpdate(const vehicle_description::State::ConstPtr& _state) {
	state.rear_wheel_speed = _state->rear_wheel_speed;
	state.steer_angle = _state->steer_angle;
}

void pathUpdate(const nav_msgs::Path::ConstPtr& _path) {
	path.header = _path->header;
	path.poses = _path->poses;
	path_ended = 0;
}

double displacement(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
	return sqrt((pose1.position.x - pose2.position.x) * (pose1.position.x - pose2.position.x) +
				(pose1.position.y - pose2.position.y) * (pose1.position.y - pose2.position.y) +
				(pose1.position.z - pose2.position.z) * (pose1.position.z - pose2.position.z));
}

double yawDifference(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
	double roll1, pitch1, yaw1;
	double roll2, pitch2, yaw2;
	
	tf::Quaternion quat1(pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w);
	tf::Quaternion quat2(pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w);
	
	tf::Matrix3x3 matrix1(quat1), matrix2(quat2);
	matrix1.getRPY(roll1, pitch1, yaw1);
	matrix2.getRPY(roll2, pitch2, yaw2);
	
	return yaw1 - yaw2;
}

void calculateParams(double &cte, double& psi) {
	double distance = displacement(pose, path.poses[0].pose);
	double min_distance = distance;
	double closest = 0;
	for (int i = 1; i < path.poses.size(); i++) {
		distance = displacement(pose, path.poses[i].pose);
		if (distance < min_distance) {
			min_distance = distance;
			closest = i;
		}
	}
	
	if (closest == path.poses.size() - 1) {
		path_ended = 1;
	}
	
	cte = min_distance; 
	psi = yawDifference(path.poses[closest].pose, pose);
}

void calculateDefaultParams(double &cte, double& psi) {
	// y = x
	
	double roll, pitch, yaw;
	tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	tf::Matrix3x3 matrix(quat);
	matrix.getRPY(roll, pitch, yaw);
	
	cte = sqrt(((pose.position.x - pose.position.y) * (pose.position.x - pose.position.y)) / 2);
	psi = PI / 4 - yaw;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
   
	// Note:
	// Can't use the approximate time policy to have a single callback function
	// The control algo should run irrespective of the incoming messages, as long as the path isn't complete
	ros::Subscriber pose_sub = n.subscribe("localization/pose", 2, poseUpdate);
	ros::Subscriber state_sub = n.subscribe("vehicle_server/state", 2, stateUpdate);
	ros::Subscriber path_sub = n.subscribe("local_planner/path", 2, pathUpdate);
	
	ros::Publisher state_pub = n.advertise<vehicle_description::State>("controller/state_command", 20);
	
	ros::Rate loop_rate(20);
	while (ros::ok()) { 
		vehicle_description::State cmd_state_msg;
		
		if (path.poses.size() == 0 || path_ended) {			
			cmd_state_msg.rear_wheel_speed = 0;
			cmd_state_msg.steer_angle = 0;
			state_pub.publish(cmd_state_msg);
		} else {		
			double cte, psi, gain = 20;
			calculateParams(cte, psi);
			//calculateDefaultParams(cte, psi);
			double cmd_steer_angle = psi + atan(gain * cte / state.rear_wheel_speed);
			
			cmd_state_msg.rear_wheel_speed = 1;
			cmd_state_msg.steer_angle = -cmd_steer_angle; // Positive -> Clockwise
			state_pub.publish(cmd_state_msg);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
