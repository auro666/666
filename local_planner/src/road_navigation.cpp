#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace message_filters;

#include "Make_traj.h"
using namespace std;

ros::Publisher bestpath_pub;

// TODO: Select the right target trajectory among lane / GPS

nav_msgs::Path decideTargetTrajectory(nav_msgs::Path::ConstPtr lane_traj) {
    return *lane_traj;
}

double getDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
    return sqrt(pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2));
}

std::vector<geometry_msgs::Pose> getTargets(geometry_msgs::Pose current_pose, nav_msgs::Path target_traj) {
    double min_distance = 5.; // In meters, along the traj
    int index = 0;
    for (; getDistance(target_traj.poses[0].pose, target_traj.poses[index].pose) < min_distance; index++);
    index--;

    geometry_msgs::Pose end;
    end.position.x = target_traj.poses[index].pose.position.x;
    end.position.y = target_traj.poses[index].pose.position.y;
    end.position.z = target_traj.poses[index].pose.position.z;
    double m = atan((current_pose.position.y - end.position.y) / (current_pose.position.x - end.position.x));
    double dist_seeds = 10;
    // Need to get laterally shifted trajectories
    std::vector<geometry_msgs::Pose> targets;
    geometry_msgs::Pose poss_target;
    for (int i = 1; i < 40; i += 1) {
        poss_target.position.x = end.position.x + i * dist_seeds * sin(m);
        poss_target.position.y = end.position.y - i * dist_seeds * cos(m);
        poss_target.position.z = end.position.z;
        targets.push_back(poss_target);
    }

    for (int i = -1; i>-40; i -= 1) {
        poss_target.position.x = end.position.x + i * dist_seeds * sin(m);
        poss_target.position.y = end.position.y - i * dist_seeds * cos(m);
        poss_target.position.z = end.position.z;
        targets.push_back(poss_target);
    }

    return targets;
}

std::vector<nav_msgs::Path> getPaths(geometry_msgs::Pose current_pose, std::vector<geometry_msgs::Pose> targets) {
    std::vector<nav_msgs::Path> paths;

    Trajectory t;

    for (int i = 0; i < targets.size(); i++) {
        paths.push_back(t.drawPath(current_pose, targets[i]));
    }

    // TODO: Prune against kinematic and dynamic constraints

    return paths;
}

void plan(const nav_msgs::Path::ConstPtr& lane_traj, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {
    nav_msgs::Path target_traj = decideTargetTrajectory(lane_traj);
    geometry_msgs::Pose current_pose = pose->pose.pose;

    std::vector<geometry_msgs::Pose> targets = getTargets(current_pose, target_traj);
    std::vector<nav_msgs::Path> paths = getPaths(current_pose, targets);

    if (paths.size() == 0) {
        ROS_WARN("[local_planner]: No path found");
        return;
    }

    nav_msgs::Path best_path;
    for (int i = 0; i < paths.size(); i++) {
        // TODO: Choose the best path
        best_path = paths[0];
    }

    best_path.header.stamp = ros::Time::now();
    bestpath_pub.publish(best_path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle n;
    bestpath_pub = n.advertise<nav_msgs::Path>("local_planner/path", 10);

    message_filters::Subscriber<nav_msgs::Path> lane_traj_sub(n, "sensor_fusion/lanes/trajectory", 10);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> localization_pose_sub(n, "localization/pose", 100);

    typedef sync_policies::ApproximateTime<nav_msgs::Path, geometry_msgs::PoseWithCovarianceStamped> ApproxTimePolicy;
    Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(100), lane_traj_sub, localization_pose_sub);

    sync.registerCallback(boost::bind(&plan, _1, _2));

    ros::spin();

    return 0;
}
