#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>

#define PI 3.141

calculateMidline(cvMat::houghImg, std::vector<cv:Vec4i> lines) {
	cv::Vec4i line;
	float m, cos, sin;
	float sumx=0,sumy=0,midx,midy,midang,sumang=0,ang,slope;
	for( int i = 0; i < lines->total; i++ )
    {
        line = lines[i];
        sumx+=(line[0] + line[2])/2;
        sumy+=(line[1] + line[3])/2;
        slope=((float)(line[3] - line[1]))/((float)(line[2] - line[0]));
        ang = atan(slope)*180/CV_PI;
        if(ang<0.0f)
            ang+=180;
        sumang+=ang;
    }
    midx=sumx/lines->total;
    midy=sumy/lines->total;
    midang=sumang/lines->total;
    m=tan(midang*CV_PI/180);
    cos=1.0/sqrt(1+m*m);
    sin=fabs(cos*m);
    if(m<0.0f)
        cos*=-1;
}

void detectLanes(const sensor_msgs::ImageConstPtr& msg) {
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
    
    //Apply Bilateral Filter to smooth keeping edges intact
    cv::Mat filtered(source.rows, source.cols, CV_8UC3);
	cv::bilateralFilter(source, filtered, 0, 21, 3);
	imshow("BilateralFiltered", filtered);
	cv::waitKey(10);
    
    //Image can be converted to YCrCb before thresholding and after thresholding it should be converted back to RGB
    
    //For thresholding calculate the mean and standard deviation in image
    cv::Scalar mean, stdDeviation, low, high;
    cv::meanStdDev(filtered, mean, stdDeviation);
    //Calculate low and high threshold for thresholding
    cv::add(mean, stdDeviation, high);
    cv::subtract(mean, stdDeviation, low);
    //Apply color masking (for color based threshold)
    cv::Mat thresholded(source.rows, source.cols, CV_8UC3);
    cv::inRange(filtered, low, high, thresholded);
    imshow("Thresholded", thresholded);
    cv::waitKey(10);
    
    //Apply Canny Edge Detection in thresholded image
    cv::Mat thresh_gray, dst, /*dstbw, */detected_edges;
    
    dst.create(thresholded.size(), CV_8UC3);
    //dstbw.create(thresholded.size(), CV_8U);
    cv::Canny(thresholded, detected_edges, 10, 30, 3);
    dst = cv::Scalar::all(0);
    filtered.copyTo(dst, detected_edges);
    imshow("Canny", dst);
    cv::waitKey(10);
    
    //Apply Hough Lines
    std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(detected_edges, lines, 1, CV_PI/180, 50, 50, 10 );
	for( size_t i = 0; i < lines.size(); i++ )
	{
		cv::Vec4i l = lines[i];
		line( dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
	}
    
    imshow("Hough", dst);
    cv::waitKey(10);
    
    //Hough lines using Vec2f
	/* std::vector<cv::Vec2f> lines;
	 * cv::HoughLines(detected_edges, lines, 1, CV_PI/180, 100, 0, 0 );
	 *
	 * for( size_t i = 0; i < lines.size(); i++ )
	 * {
	 *		float rho = lines[i][0], theta = lines[i][1];
	 *		cv::Point pt1, pt2;
	 *		double a = cos(theta), b = sin(theta);
	 *		double x0 = a*rho, y0 = b*rho;
	 *		pt1.x = cvRound(x0 + 1000*(-b));
	 *		pt1.y = cvRound(y0 + 1000*(a));
	 *		pt2.x = cvRound(x0 - 1000*(-b));
	 *		pt2.y = cvRound(y0 - 1000*(a));
	 *		line( dst, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
	 *	}
	 *	imshow("Hough", dst);
	 *	cv::waitKey(10);
     */
     
     //Calculate midline
     calculateMidline(dst, lines);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lane_detection");
	ros::NodeHandle nh;
	
	cv::namedWindow("Source", 0);
	cv::namedWindow("BilateralFiltered", 0);
	cv::namedWindow("Thresholded", 0);
	cv::namedWindow("Canny", 0);
	cv::namedWindow("Hough", 0);
	
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("logitech/left/image", 2, detectLanes);
	
	ros::spin();

	return 0;
}
