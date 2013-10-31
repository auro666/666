#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vehicle_description/State.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <iostream>
#include <cmath>
#include <pthread.h>

#define PID_P 50
#define PID_I 15
#define PID_D 10
#define PID_I_MAX 50
#define PID_I_MIN -50
#define CMD_MAX 30
#define CMD_MIN -30

#define PI 3.14159265359

#define FRONT_LEFT_YAW 0
#define FRONT_RIGHT_YAW 1
#define BACK_LEFT_VEL 2
#define BACK_RIGHT_VEL 3

// Vehicle Parameters
#define wheelRadius 0.3
#define wheelBase 1.2
#define vehicleLength 1.6

pthread_mutex_t vel_lock;
pthread_mutex_t theta_lock;

double cmd_vel = 0, cur_vel = 0;
double cmd_theta = 0, cur_theta = 0;
double target_x, target_y;
double bot_x, bot_y, bot_z, qx, qy, qz, qw;

void init() {
	if (pthread_mutex_init(&vel_lock, NULL) != 0) {
		throw "Mutex init failed";
	}
	
	if (pthread_mutex_init(&theta_lock, NULL) != 0) {
		throw "Mutex init failed";
	}
	
	cmd_theta = 0;
	cmd_vel = 0;
	cur_theta = cur_vel = 0;
}

void commandState(const vehicle_description::State::ConstPtr& msg) {
	cmd_theta = msg->steer_angle;
	cmd_vel = msg->rear_wheel_speed;
}

int getPose(const gazebo::math::Pose data) {
	bot_x = data.pos.x;
	bot_y = data.pos.y;
	bot_z = data.pos.z;
	
	qx =data.rot.x;
	qy =data.rot.y;
	qz =data.rot.z;
	qw =data.rot.w;
	
	return 0;
}

namespace gazebo {
	class JointControl : public ModelPlugin {
		private:
			double VEL_PID_P, VEL_PID_I, VEL_PID_D, VEL_PID_I_MAX, VEL_PID_I_MIN, VEL_CMD_MAX, VEL_CMD_MIN;
			gazebo::common::PID pid[4];
			gazebo::physics::JointPtr joint[4], refjoint_1, refjoint_2;
			double targets[4];
			gazebo::physics::ModelPtr model_;
			gazebo::event::ConnectionPtr update_connection_;
			gazebo::event::ConnectionPtr update_connection_2;
			gazebo::common::Time last_update_time_;
			int temp;
			double error;
			double dt;
			gazebo::math::Vector3 location;
			gazebo::math::Vector3 dummy;
			double tanTheta;
			
			ros::Publisher pose_pub;
			ros::Publisher state_pub;
			ros::Subscriber state_sub;
			
			geometry_msgs::Pose _pose;
			vehicle_description::State _state;
			
		public: 
			void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
				init();
				char *argv[] = {"TEST"};
				int argc = 1;
				ros::init(argc, argv, "Plugin");
				ros::NodeHandle n;

				state_sub = n.subscribe("controller/state_command", 2, commandState);
				pose_pub = n.advertise<geometry_msgs::Pose>("simulator/pose", 100);
				state_pub = n.advertise<vehicle_description::State>("simulator/state", 100);

				this->model_ = _model;

				this->VEL_PID_P     = _sdf->Get<double>("PID_P");
				this->VEL_PID_I     = _sdf->Get<double>("PID_I");
				this->VEL_PID_D     = _sdf->Get<double>("PID_D");
				this->VEL_PID_I_MAX = _sdf->Get<double>("PID_I_MAX");
				this->VEL_PID_I_MIN = _sdf->Get<double>("PID_I_MIN");
				this->VEL_CMD_MAX   = _sdf->Get<double>("PID_CMD_MAX");
				this->VEL_CMD_MIN   = _sdf->Get<double>("PID_CMD_MIN");

				/* *************** Initialising the PID controls ****************** */
				for (temp = 0; temp < 2; temp++) {
					this->pid[temp].Init(PID_P, PID_I, PID_D, PID_I_MAX, PID_I_MIN, CMD_MAX, CMD_MIN);
					this->pid[temp].SetCmd(0);
				}
				
				for (temp = 2; temp < 4; temp++) {
					this->pid[temp].Init(VEL_PID_P, VEL_PID_I, VEL_PID_D, VEL_PID_I_MAX, VEL_PID_I_MIN, VEL_CMD_MAX, VEL_CMD_MIN);
					this->pid[temp].SetCmd(0);
				}

				/* ********************* Assigning the joint pointers *********************** */
				this->joint[FRONT_LEFT_YAW]   = this->model_->GetJoint("front_left_joint");
				this->joint[FRONT_RIGHT_YAW]  = this->model_->GetJoint("front_right_joint");
				this->joint[BACK_LEFT_VEL]    = this->model_->GetJoint("back_left_joint");
				this->joint[BACK_RIGHT_VEL]   = this->model_->GetJoint("back_right_joint");
				this->refjoint_1   = this->model_->GetJoint("ref_1_joint");
				this->refjoint_2   = this->model_->GetJoint("ref_2_joint");


				this->last_update_time_ = this->model_->GetWorld()->GetSimTime();
				this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&JointControl::UpdateCMD, this));
		}

			void UpdateCMD() {
				ros::spinOnce();
				targets[BACK_LEFT_VEL] = cmd_vel / wheelRadius;
				targets[BACK_RIGHT_VEL] = cmd_vel / wheelRadius;
				
				// Something wrong here! serisously wrong!
				tanTheta = tan(cmd_theta);
				targets[FRONT_LEFT_YAW]  = atan2(2 * vehicleLength * tanTheta, 2 * vehicleLength - wheelBase * tanTheta);
				targets[FRONT_RIGHT_YAW] = atan2(2 * vehicleLength * tanTheta, 2 * vehicleLength + wheelBase * tanTheta);
				
				gazebo::common::Time current_time = this->model_->GetWorld()->GetSimTime();
				for (this->temp = 0; this->temp < 4; this->temp++) {
					if (this->temp == 0 || this->temp == 1) {
						error = this->joint[this->temp]->GetAngle(0).Radian() - targets[this->temp];
					}
					
					if (this->temp == 2 || this->temp == 3) {
						error = this->joint[this->temp]->GetVelocity(0) - targets[this->temp];
					}
					
					dt = current_time.Double() - this->last_update_time_.Double();
					this->pid[this->temp].Update(error, dt);
					this->joint[this->temp]->SetForce(0, this->pid[this->temp].GetCmd());
				}
				this->last_update_time_ = current_time;
				
				tanTheta = this->joint[FRONT_LEFT_YAW]->GetAngle(0).Radian();
				cur_theta = atan(2 * vehicleLength * tanTheta / (2 * vehicleLength + wheelBase * tanTheta));
				cur_vel = this->joint[BACK_LEFT_VEL]->GetVelocity(0) * wheelRadius;
				
				location = this->refjoint_1->GetAnchor(1);
				dummy    = this->refjoint_2->GetAnchor(1);
				
				getPose(this->model_->GetWorldPose());
				
				_pose.position.x = location.x;
				_pose.position.y = location.y;
				_pose.position.z = location.z;
				
				_pose.orientation.x = qx;
				_pose.orientation.y = qy;
				_pose.orientation.z = qz;
				_pose.orientation.w = qw;
				
				pose_pub.publish(_pose);
				
				_state.steer_angle = cur_theta;
				_state.rear_wheel_speed = cur_vel;
				state_pub.publish(_state);
			}
	};

	GZ_REGISTER_MODEL_PLUGIN(JointControl)
}

