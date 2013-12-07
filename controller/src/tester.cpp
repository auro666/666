#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <vehicle_description/State.h>
#include <std_msgs/Float64.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#define PI 3.141

geometry_msgs::Pose pose;
vehicle_description::State state;

int debug_flag = 1;

std::vector<int> cte_plot_data;
std::vector<cv::Point> cte_plot;
std::vector<cv::Point> flat_line;
cv::Mat cte_image;

std::vector<cv::Point> path, target;
cv::Mat path_image;

void poseUpdate(const geometry_msgs::Pose::ConstPtr& _pose) {
	pose.position = _pose->position;
	pose.orientation = _pose->orientation;
	
	double roll, pitch, yaw;
	tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	tf::Matrix3x3 matrix(quat);
	matrix.getRPY(roll, pitch, yaw);
}

void stateUpdate(const vehicle_description::State::ConstPtr& _state) {
	state.rear_wheel_speed = _state->rear_wheel_speed;
	state.steer_angle = _state->steer_angle;
}

void plotCTE(const std_msgs::Float64::ConstPtr& _cte) {
	double cte = _cte->data * 100;
	if (debug_flag) {
		cte_plot_data.erase(cte_plot_data.begin());
		cte_plot_data.push_back(300 + cte);
		cte_plot.clear();
		for (int i = 0; i < cte_plot_data.size(); i++) {
			cte_plot.push_back(cv::Point(i, cte_plot_data[i]));
		}
		
		cte_image = cv::Scalar(0, 0, 0);
		for (int i = 0; i < cte_plot.size() - 1; i++) {
			cv::line(cte_image, cte_plot[i], cte_plot[i + 1], cv::Scalar(0, 255, 255), 1);                        
		}
		for (int i = 0; i < flat_line.size() - 1; i++) {
			cv::line(cte_image, flat_line[i], flat_line[i + 1], cv::Scalar(255, 0, 255), 1);                        
		}
		
		cv::imshow("CTE", cte_image);
		cv::waitKey(1);
	}	
}

void plotPath() {
	if (debug_flag) {
		path_image = cv::Scalar(0, 0, 0);
		path.push_back(cv::Point(10 + pose.position.x, 300 - pose.position.y));
		for (int i = 0; i < path.size() - 1; i++) {
			cv::line(path_image, path[i], path[i + 1], cv::Scalar(0, 255, 255), 1);                        
		}
		for (int i = 0; i < target.size() - 1; i++) {
			cv::line(path_image, target[i], target[i + 1], cv::Scalar(255, 0, 255), 1);                        
		}
		imshow("Path", path_image);
		cv::waitKey(1);
	}
}

void selectPath(nav_msgs::Path& path_msg) {
	int path_type = 2;
	switch(path_type) {
		case 0: {
			int n_poses = 1000;
			path_msg.poses.resize(n_poses);
			for (int i = 0; i < path_msg.poses.size(); i++) {
				path_msg.poses[i].pose.position.x = i;
				path_msg.poses[i].pose.position.y = i;
				path_msg.poses[i].pose.position.z = 0;	
				path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
				
				target.push_back(cv::Point(10 + i, 300 + i));
			}
    }
			break;
		case 1: {
			int n_poses = 1000;
			path_msg.poses.resize(n_poses);
			for (int i = 0; i < path_msg.poses.size(); i++) {
				path_msg.poses[i].pose.position.x = i;
				path_msg.poses[i].pose.position.y = sin(i / 100.) * 100;
				path_msg.poses[i].pose.position.z = 0;	
				path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, atan(cos(i / 100.)));
				
				target.push_back(cv::Point(10 + i, 300 - sin(i / 100.) * 100));
			}
    }
			break;
		case 2: {
			std::ifstream curve_file ("../data/curve.txt");
			if (curve_file.is_open()) {
				std::string line;
				std::getline(curve_file, line);
				std::stringstream ss(line);
				int n_poses;
				if (ss >> n_poses) {
					path_msg.poses.resize(n_poses);
					for (int i = 0; i < path_msg.poses.size(); i++) {
						std::getline(curve_file, line);
						double x, y, theta;
						std::stringstream ss(line);
						ss >> x >> y >> theta;
						
						path_msg.poses[i].pose.position.x = x;
						path_msg.poses[i].pose.position.y = y;
						path_msg.poses[i].pose.position.z = 0;	
						path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta);
						
						target.push_back(cv::Point(10 + x, 300 - y));
					}
				}
				curve_file.close();
			}
    }
			break;
		default:
			break;
	}
}

void setupCTEDisplay() {
	cte_image.create(600, 2000, CV_8UC3);
	cv::namedWindow("CTE", 0);
	for (int i=0; i < 2000; i++) {
		cte_plot_data.push_back(300);
		flat_line.push_back(cv::Point(i, 300));
	}
}

void setupPathDisplay() {
	path_image.create(600, 800, CV_8UC3);
	cv::namedWindow("Path", 0);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "tester");
	ros::NodeHandle n;

	// Observed stuff from simulator. Noise can be added later.
	ros::Subscriber state_sub = n.subscribe("simulator/state", 2, stateUpdate);
	ros::Subscriber pose_sub = n.subscribe("simulator/pose", 2, poseUpdate);
	
	// Feedback from controller for perf analysis
	ros::Subscriber cte_sub = n.subscribe("controller/cross_track_error", 2, plotCTE);
	
	// Mimics other nodes
	ros::Publisher state_pub = n.advertise<vehicle_description::State>("vehicle_server/state", 20);
	ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("localization/pose", 100);
	ros::Publisher path_pub = n.advertise<nav_msgs::Path>("local_planner/path", 10);
	
	setupCTEDisplay();
	setupPathDisplay();
	
	nav_msgs::Path path_msg;
	selectPath(path_msg);

	ros::Rate loop_rate(100);
	int count = 0;
	while (ros::ok()) {
		n.getParam("/tester/debug_mode", debug_flag);

		state_pub.publish(state);
		pose_pub.publish(pose);
		
		path_msg.header.seq = count;
		path_msg.header.frame_id = count;
		path_msg.header.stamp = ros::Time::now();
		path_pub.publish(path_msg);
		plotPath();

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}
