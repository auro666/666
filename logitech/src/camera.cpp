#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#define LEFT_CAM 0
#define RIGHT_CAM 1

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_pub");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher left_pub = it.advertise("stereo/left/image_raw", 30);
	image_transport::Publisher right_pub = it.advertise("stereo/right/image_raw", 30);
	sensor_msgs::ImagePtr msg;
	
	CvCapture* left_capture = 0;
	CvCapture* right_capture = 0;
    left_capture = cvCaptureFromCAM(LEFT_CAM); // read AVI video
    right_capture = cvCaptureFromCAM(RIGHT_CAM); // read AVI video
	if (!left_capture || !right_capture) {
		throw "Unable to capture from stereo camera";
	}
    
    ros::Rate loop_rate(30);
	IplImage* left_frame = 0;
    IplImage* right_frame = 0;
    while (nh.ok()) {
		left_frame = cvQueryFrame(left_capture);
		right_frame = cvQueryFrame(right_capture);
		if (!left_frame || !right_frame) {    
			throw "Unable to get stereo camera frame";
			break;
		}
		
		left_pub.publish(sensor_msgs::CvBridge::cvToImgMsg(left_frame, "bgr8"));
		right_pub.publish(sensor_msgs::CvBridge::cvToImgMsg(right_frame, "bgr8"));
		loop_rate.sleep();
	}
	cvReleaseImage(&left_frame);
	cvReleaseImage(&right_frame);
	return(0);
}
