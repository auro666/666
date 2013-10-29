#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "right_camera_pub");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("logitech/right/image", 10);
	
	cv::VideoCapture cap(1); 
    if(!cap.isOpened()) {
        throw "Error when reading from right camera";
	}
	
    cv::namedWindow("Left Camera", 1);
    ros::Rate loop_rate(10);
    int i = 0;
	while (nh.ok()) {
		cv::Mat frame;
		cap >> frame;
		imshow("Right Camera", frame);
		cv::waitKey(1);
		
		cv_bridge::CvImage msg;
		msg.header.seq = i;
		msg.header.frame_id = i;
		msg.header.stamp = ros::Time::now();
		msg.encoding = sensor_msgs::image_encodings::BGR8;
		msg.image = frame;
		
		pub.publish(msg.toImageMsg());
		loop_rate.sleep();
	}
	
	return 0;
}
