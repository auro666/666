#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "left_camera_pub");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("stereo/left/image", 10);
	sensor_msgs::ImagePtr msg;
	
	CvCapture* capture = 0;
    IplImage* frame = 0;
    capture = cvCaptureFromCAM(0); // read AVI video    
	if (!capture) {
		throw "Error when reading steam_avi";
	}
    
    ros::Rate loop_rate(10);
	while (nh.ok()) {
		frame = cvQueryFrame(capture);
		if (!frame) {    
			throw "Unable to get frame";
			break;
		}
		
		msg = sensor_msgs::CvBridge::cvToImgMsg(frame, "bgr8");
		pub.publish(msg);
		loop_rate.sleep();
	}
	cvReleaseImage(&frame);
	return(0);
}
