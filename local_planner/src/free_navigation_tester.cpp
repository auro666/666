#include <ros/ros.h>
#include <local_planner/MapUpdate.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>

unsigned char *map;
int num_cols = 4000, num_rows = 4000;

bool mapServer(local_planner::MapUpdate::Request& req, local_planner::MapUpdate::Response& res) {
    int sensing_width = 5; // Meters
    int sensing_range = sensing_width / req.resolution;

    res.sensing_range = sensing_range;
    res.snippet = new unsigned char [(2 * sensing_range + 1) * (2 * sensing_range + 1)];
    res.valid = false;

    for (int x = -sensing_range; x <= sensing_range; x++) {
        for (int y = -sensing_range; y <= sensing_range; y++) {
            int map_x = x + req.pose.position.x;
            int map_y = y + req.pose.position.y;
            if ((0 <= map_x) && (map_x < num_rows) && (0 <= map_y) && (map_y < num_cols)) {
                int map_index = map_x + map_y * num_rows;
                int snippet_index = (x + sensing_range) + (y + sensing_range) * (2 * sensing_range + 1);
                res.snippet[snippet_index] = map[map_index];
                res.valid = true;
            }
        }
    }
}

void displayMap() {
    cv::Mat map_frame(num_rows, num_cols, CV_8UC1, map);
    cv::Mat display_frame;
    cv::resize(map_frame, display_frame, 0, 1. / 8, 1. / 8, CV_INTER_AREA);
    imshow("Map", display_frame);
    cv::waitKey(10);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "free_navigation_tester");
    ros::NodeHandle n;
    cv::namedWindow("Map", 1);

    map = new unsigned char [num_cols * num_rows];

    ros::ServiceServer map_service = n.advertiseService("map_service", mapServer);

    while (ros::ok()) {
        displayMap();
        ros::spinOnce();
    }
}
