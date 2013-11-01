// Subscribes to state, pose
// Has hard coded trajectory (along x axis - [0, 0, 0], [1, 0, 0], etc)
// Publishes state, pose, path

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <vehicle_description/State.h>
#define PI 3.141

geometry_msgs::Pose pose;
vehicle_description::State state;
double error;
int n = 0;

/*
class plot{
public: 
int width;
std::vector<CvPoint> curve;
std::vector<CvPoint> tolerance_curve_p;
std::vector<CvPoint> tolerance_curve_n;
std::vector<int> data;
double tolerance_range_p;
double tolerance_range_n;
char name[1024];
IplImage *image;
double scale;
int height;
bool tolerance;
plot(int w,char *n, double s, int height_ = 100, bool tolerance_=false, double tolerance_range_p_=0, double tolerance_range_n_=0){
scale=s;
height = height_;
tolerance = tolerance_;
image = cvCreateImage( cvSize(w,height), 8, 3 );
width=w;
snprintf(name, sizeof(name), n);
cvNamedWindow( name, 1 );
data.clear();
curve.clear();
tolerance_curve_n.clear();
tolerance_curve_p.clear();
if(tolerance == true){
tolerance_range_p = tolerance_range_p_;
tolerance_range_n = tolerance_range_n_;
}
for(int i=0;i<w;i++){
data.push_back(height/2);
if(tolerance == true){
tolerance_curve_p.push_back(cvPoint(i,-1*tolerance_range_p*scale + height/2));
tolerance_curve_n.push_back(cvPoint(i,-1*tolerance_range_n*scale + height/2));
}
}
}
void show(){
curve.clear();
cvSet(image, cvScalar(0,0,0));
for(int i=0;i<data.size();i++){
curve.push_back(cvPoint(i,data[i]));
}
for (int i = 0; i < curve.size()-1; i++){
cvLine(image, curve[i], curve[i+1], cvScalar(0,255,255), 1);                        
if(tolerance == true){
cvLine(image, tolerance_curve_p[i], tolerance_curve_p[i+1], cvScalar(255,0,0), 1);                        
cvLine(image, tolerance_curve_n[i], tolerance_curve_n[i+1], cvScalar(255,0,0), 1);                        
}
}
cvShowImage(name,image);
}
void add_data(double d){
data.erase(data.begin());
data.push_back(-1*d*scale + height/2);
show();
}
};*/
//plot *p;
void poseUpdate(const geometry_msgs::Pose::ConstPtr& _pose) {
	pose.position = _pose->position;
	pose.orientation = _pose->orientation;
	
	double roll, pitch, yaw;
	tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	tf::Matrix3x3 matrix(quat);
	matrix.getRPY(roll, pitch, yaw);
	
	ROS_INFO("True Pose2D: [%.2lf, %.2lf, %.2lf]", pose.position.x, pose.position.y, yaw * 180 / PI);
	error = (n*error + sqrt(((pose.position.x - pose.position.y) * (pose.position.x - pose.position.y)) / 2)) / (n++ + 1);
	ROS_INFO("TrajFollow Error: [%.2lf]", error);
	//p.add_data(error);
}

void stateUpdate(const vehicle_description::State::ConstPtr& _state) {
	state.rear_wheel_speed = _state->rear_wheel_speed;
	state.steer_angle = _state->steer_angle;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	//p = new plot(500,"Error",100, 300);
	// Observed stuff from simulator. Noise can be added later.
	ros::Subscriber state_sub = n.subscribe("simulator/state", 2, stateUpdate);
	ros::Subscriber pose_sub = n.subscribe("simulator/pose", 2, poseUpdate);
	
	// Mimics other nodes
	ros::Publisher state_pub = n.advertise<vehicle_description::State>("vehicle_server/state", 20);
	ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("localization/pose", 100);
	ros::Publisher path_pub = n.advertise<nav_msgs::Path>("local_planner/path", 10);
	
	int n_poses = 1000;
	nav_msgs::Path path_msg;
	path_msg.poses.resize(n_poses);
	for (int i = 0; i < path_msg.poses.size(); i++) {
		path_msg.poses[i].pose.position.x = i;
		path_msg.poses[i].pose.position.y = i;
		path_msg.poses[i].pose.position.z = 0;	
		path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -PI / 4);
	}
	
	ros::Rate loop_rate(100);
	int count = 0;
	error = 0;
	while (ros::ok()) {
		state_pub.publish(state);
		pose_pub.publish(pose);
		
		path_msg.header.seq = count;
		path_msg.header.frame_id = count;
		path_msg.header.stamp = ros::Time::now();
		path_pub.publish(path_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}
