#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define PI 3.141
#define HEIGHT 800
cv::Mat img(cvSize(800, 800), CV_8UC3, cvScalarAll(0));

void selectPath(nav_msgs::Path& path_msg) {
    int n_poses = 1000;
    path_msg.poses.resize(n_poses);
    for (int i = 0; i < path_msg.poses.size(); i++) {
        path_msg.poses[i].pose.position.x = 400;
        path_msg.poses[i].pose.position.y = 400;
        path_msg.poses[i].pose.position.z = PI / 4;
    }
}

void generate_pos(geometry_msgs::PoseWithCovarianceStamped& current_pos) {

    current_pos.pose.pose.position.x = 300;
    current_pos.pose.pose.position.y = 300;
    current_pos.pose.pose.position.z = 0;

}

void bestPath(const nav_msgs::Path::ConstPtr& path_msg) {

    cv::circle(img, cvPoint(path_msg->poses[0].pose.position.x, HEIGHT - path_msg->poses[0].pose.position.y), 5, cvScalarAll(255));
    cv::circle(img, cvPoint(path_msg->poses[path_msg->poses.size() - 1].pose.position.x, HEIGHT - path_msg->poses[path_msg->poses.size() - 1].pose.position.y), 5, cvScalarAll(255));
    cv::line(img, cvPoint(path_msg->poses[0].pose.position.x, HEIGHT - path_msg->poses[0].pose.position.y), cvPoint(path_msg->poses[path_msg->poses.size() - 1].pose.position.x, HEIGHT - path_msg->poses[path_msg->poses.size() - 1].pose.position.y), cvScalar(255));
    for (int i = 0; i < path_msg->poses.size() - 1; i++) {
        cv::line(img, cvPoint(path_msg->poses[i].pose.position.x, HEIGHT - path_msg->poses[i].pose.position.y), cvPoint(path_msg->poses[i + 1].pose.position.x, HEIGHT - path_msg->poses[i + 1].pose.position.y), cvScalar(255, 255, 255));
    }

    cv::imshow("Path Planned B)", img);
    cvWaitKey(0);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tester");
    ros::NodeHandle n;
    nav_msgs::Path path_msg;

    geometry_msgs::PoseWithCovarianceStamped current_pos;
    generate_pos(current_pos);
    selectPath(path_msg);
    ros::Publisher lanetraj_pub = n.advertise<nav_msgs::Path>("sensor_fusion/lanes/trajectory", 10);
    ros::Publisher pos_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("localization/pose", 100);
    ros::Subscriber bestpath_sub = n.subscribe("local_planner/path", 2, bestPath);

    ros::Rate loop_rate(100);
    int count = 0;
    while (ros::ok()) {
        path_msg.header.seq = count;
        path_msg.header.frame_id = count;
        path_msg.header.stamp = ros::Time::now();
        current_pos.header.seq = count;
        current_pos.header.frame_id = count;
        current_pos.header.stamp = ros::Time::now();
        pos_pub.publish(current_pos);
        lanetraj_pub.publish(path_msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }



    return 0;
}
