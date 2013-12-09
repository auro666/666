#include <ros/ros.h>
#include <local_planner/MapUpdate.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_datatypes.h>

void updateMap(int event, int x, int y, int flags, void *param);

class Tester {
private:
    geometry_msgs::Pose current_pose;
    unsigned char *map;
    ros::NodeHandle n;
    ros::ServiceServer map_service;
    ros::Publisher pose_pub;
    ros::Publisher goal_pub;
    ros::Subscriber path_sub;

    // TODO: Implement this in a dedicated map server

    bool isValidPoint(int x, int y) {
        return (0 <= x) && (x < num_rows) && (0 <= y) && (y < num_cols);
    }

    int calculateIndex(int x, int y) {
        return x + y * num_rows;
    }

    bool serveSnippets(local_planner::MapUpdate::Request& req, local_planner::MapUpdate::Response& res) {
        int sensing_range = 20;
        res.sense_range = sensing_range;
        res.snippet.resize((2 * sensing_range + 1) * (2 * sensing_range + 1), 0);
        res.valid_flags.resize((2 * sensing_range + 1) * (2 * sensing_range + 1), false);

        for (int x = -sensing_range; x <= sensing_range; x++) {
            for (int y = -sensing_range; y <= sensing_range; y++) {
                int map_x = x + req.pose.position.x;
                int map_y = y + req.pose.position.y;
                int snippet_index = (x + sensing_range) + (y + sensing_range) * (2 * sensing_range + 1);
                if (isValidPoint(map_x, map_y)) {
                    res.snippet.at(snippet_index) = map[calculateIndex(map_x, map_y)];
                    res.valid_flags.at(snippet_index) = true;
                }
            }
        }

        return true;
    }

    void updateCurrentPose(const nav_msgs::Path::ConstPtr& path) {
        if (path->poses.size() > 1) {
            current_pose = path->poses.at(path->poses.size() / 10).pose;
            ROS_INFO(
                    "[freeform_navigation_tester] : Current Pose Updated : (%lf, %lf, %lf)",
                    current_pose.position.x,
                    current_pose.position.y,
                    tf::getYaw(current_pose.orientation));
        }
    }

public:
    int num_rows, num_cols;

    Tester() {
        num_rows = num_cols = 400;
        map = new unsigned char [num_cols * num_rows];
        for (int i = 0; i < num_cols * num_rows; i++) {
            map[i] = 0;
        }

        current_pose.position.x = 5;
        current_pose.position.y = 1;
        current_pose.orientation = tf::createQuaternionMsgFromYaw(0);

        map_service = n.advertiseService("map_service", &Tester::serveSnippets, this);
        pose_pub = n.advertise<geometry_msgs::Pose>("localization/pose", 100);
        goal_pub = n.advertise<geometry_msgs::Pose>("master_planner/next_way_point", 1);
        path_sub = n.subscribe("local_planner/path", 2, &Tester::updateCurrentPose, this);
        cv::namedWindow("Map", 0);
        cv::setMouseCallback("Map", updateMap, this);
    }

    void fillSquare(int x, int y, unsigned char value) {
        int window_size = 20; // Pixels
        for (int i = -window_size; i <= window_size; i++) {
            for (int j = -window_size; j <= window_size; j++) {
                if (isValidPoint(x + i, y + j)) {
                    map[calculateIndex(x + i, y + j)] = value;
                }
            }
        }
    }

    void publishNewGoal(int x, int y) {
        geometry_msgs::Pose new_goal;
        new_goal.position.x = x;
        new_goal.position.y = y;
        new_goal.orientation = tf::createQuaternionMsgFromYaw(0);
        goal_pub.publish(new_goal);
    }

    void display() {
        cv::Mat map_frame(num_rows, num_cols, CV_8UC1, map);
        cv::circle(
                map_frame,
                cv::Point(current_pose.position.x / 0.025, current_pose.position.y / 0.025),
                1,
                cv::Scalar(128, 128, 128),
                1,
                CV_AA,
                0);
        cv::imshow("Map", map_frame);
        cv::waitKey(10);
    }

    void sendCurrentPose() {
        pose_pub.publish(current_pose);
    }
};

void updateMap(int event, int x, int y, int flags, void *param) {
    Tester *tester = (Tester *) param;
    switch (event) {
        case CV_EVENT_LBUTTONDOWN:
            // Obstacle
            tester->fillSquare(x, y, 255);
            break;
        case CV_EVENT_RBUTTONDOWN:
            // Goal
            tester->publishNewGoal(x, y);
            break;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "freeform_navigation_tester");
    Tester tester = Tester();

    while (ros::ok()) {
        tester.sendCurrentPose();
        tester.display();

        ros::spinOnce();
    }
}
