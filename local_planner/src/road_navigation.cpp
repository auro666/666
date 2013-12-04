#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

using namespace message_filters;

nav_msgs::Path decideTargetTrajectory(nav_msgs::Path::ConstPtr lane_traj) {
	return *lane_traj;
}

double getDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
	return sqrt(pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2));
}

std::vector<geometry_msgs::Pose> getTargets(nav_msgs::Path target_traj) {
	double min_distance = 5.; // In meters, along the traj
	int index = 0;
	for (; getDistance(target_traj.poses[index].pose, target_traj.poses[index].pose) < min_distance; index++);
	index--;
	
	// Need to get laterally shifted trajectories
	std::vector<geometry_msgs::Pose> targets;
	targets.push_back(target_traj.poses[index].pose);
	
	return targets;
}

nav_msgs::Path generateTrajectory(geometry_msgs::Pose current_pose, geometry_msgs::Pose target_pose) {
	nav_msgs::Path path;
	// Generate Euler Spiral with zero boundary curvatures
	return path;
}

std::vector<nav_msgs::Path> getPaths(geometry_msgs::Pose current_pose, std::vector<geometry_msgs::Pose> targets) {
	std::vector<nav_msgs::Path> paths;
	for (int i = 0; i < targets.size(); i++) {
		paths.push_back(generateTrajectory(current_pose, targets[i]));
	}
	// Prune against kinematic and dynamic constraints
	return paths;
}

void plan(const nav_msgs::Path::ConstPtr& lane_traj, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {
	nav_msgs::Path target_traj = decideTargetTrajectory(lane_traj);
	geometry_msgs::Pose current_pose = pose->pose.pose;
	
	std::vector<geometry_msgs::Pose> targets = getTargets(target_traj);
	std::vector<nav_msgs::Path> paths = getPaths(current_pose, targets);
	
	if (paths.size() == 0) {
		ROS_WARN("[local_planner]: No path found");
		return;
	}
	
	nav_msgs::Path best_path;
	for (int i = 0; i < paths.size(); i++) {
		// Choose the best path
	}
	
	// Publish the best path
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "local_planner");
	ros::NodeHandle n;
	
	message_filters::Subscriber<nav_msgs::Path> lane_traj_sub(n, "sensor_fusion/lanes/trajectory", 10);
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> localization_pose_sub(n, "localization/pose", 100);

	typedef sync_policies::ApproximateTime<nav_msgs::Path, geometry_msgs::PoseWithCovarianceStamped> ApproxTimePolicy;
	Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(100), lane_traj_sub, localization_pose_sub);
	
	sync.registerCallback(boost::bind(&plan, _1, _2));
  
	ros::spin();

	return 0;
}
