#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "transport2.h"

image_transport::Publisher points_pub;
int seq_id;

void callback(PointCloud points) {
	cv::Mat point_cloud(points.height, points.width, CV_8U, points.data);
	cv_bridge::CvImage msg;
	msg.header.seq = seq_id;
	msg.header.frame_id = seq_id;
	msg.header.stamp = ros::Time::now();
	msg.encoding = sensor_msgs::image_encodings::MONO8;
	msg.image = point_cloud;
	
	points_pub.publish(msg.toImageMsg());
	ROS_INFO("[stereo_publisher]: Lag - %u", time(0) - points.stamp);
	seq_id++;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "stereopsis_publisher");
    ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	points_pub = it.advertise("stereo/points", 10);
    cv::namedWindow("Point Cloud", 0);
	
    Subscriber<PointCloud> sub(6007, callback);
	seq_id = 0;
	while (ros::ok()) {
		usleep(10);
	}
	
	sub.finish();
	return 0;
}
