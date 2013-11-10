#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>

#define PI 3.141

void callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Subscribed image in source
    cv::Mat source = cv_ptr->image;
    imshow("Source", source);
    cv::waitKey(10);
}

void mouseEvent(int evt, int x, int y, int flags, void* param) {
    if(evt == CV_EVENT_LBUTTONDOWN){
        ROS_INFO("%d %d", x, y);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lane_detection");
    ros::NodeHandle nh;

    cv::namedWindow("Source", 0);
    cv::namedWindow("BilateralFiltered", 0);
    cv::namedWindow("Thresholded", 0);
    cv::namedWindow("Canny", 0);
    cv::namedWindow("Hough", 0);

	cv::setMouseCallback("Source", mouseEvent, 0);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("logitech/left/image", 2, callback);

    ros::spin();

    return 0;
}
