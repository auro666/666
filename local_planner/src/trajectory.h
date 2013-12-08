// TODO: Convert all print statements to ROS_DEBUG / ROS_INFO

/* 
 * File:   Make_traj.h
 * Author: shiwangi
 *
 * Created on 6 December, 2013, 5:21 PM
 */
#ifndef TRAJECTORY_H
#define	TRAJECTORY_H
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include "clothoid.hpp"

class Poses {
public:
    State a, b;

    Poses(State a_, State b_) {
        a = a_;
        b = b_;
    }
};

class Trajectory {
public:
    double inRange(double theta);
    double actualAtan(double theta, State a, State b);
    nav_msgs::Path drawPath(geometry_msgs::Pose current_pose, geometry_msgs::Pose target_pose);

};

double Trajectory::inRange(double theta) {
    if (theta > PI) {
        while (theta > PI) {
            theta -= 2 * PI;
        }
    }

    if (theta < PI) {
        while (theta <= -PI) {
            theta += 2 * PI;
        }
    }

    return theta;
}

//To get the angle in the range from 0 to 2*PI

double Trajectory::actualAtan(double theta, State a, State b) {
    if ((b.x - a.x) < 0) {
        if ((b.y - a.y) < 0) {
            theta = PI + theta;
        } else {
            theta += PI;
        }
    } else if (theta < 0) {
        theta += 2 * PI;
    }

    return theta;
}

nav_msgs::Path Trajectory::drawPath(geometry_msgs::Pose current_pose, geometry_msgs::Pose target_pose) {
    State a(current_pose.position.x, current_pose.position.y, current_pose.position.z);
    State b(target_pose.position.x, target_pose.position.y, target_pose.position.z);
    std::vector<State> path;
    Clothoid curve;
    nav_msgs::Path paths;
    double beta = atan((b.y - a.y) / (b.x - a.x));

    //printing the parameters.
    ROS_DEBUG("start point : (%lf, %lf, %lf)", a.x, a.y, a.theta);
    //std::cout << "end point : (" << b.x << "," << b.y << "," << b.theta << ")" << std::endl << std::endl;
    //std::cout << "Angle between the start and end points : " << beta << std::endl << std::endl;

    //if the symmetry condition is specified we get an elementary Euler curve--- Ref: A star Spatial
    if (fabs(beta - b.theta - a.theta + beta) < 0.001) {
        curve.getControls(a, b);
        curve.getTrajectory();
        path = curve.plotPath();
        //std::cout << std::endl << std::endl << std::endl;
    } else if (a.theta == b.theta) { //when the start and end points have the same direction vector.
        //2 euler curves via point p.
        State p((a.x + b.x) / 2, (a.y + b.y) / 2, 0);
        double beta = atan((p.y - a.y) / (p.x - a.x));
        p.theta = 2 * beta - a.theta;
        //cout << p.theta << endl;

        //Plotting Euler curves between a&p && p&b.
        curve.getControls(a, p);
        curve.getTrajectory();
        path = curve.plotPath();
        curve.getControls(p, b);
        curve.getTrajectory();

        std::vector<State> another_path = curve.plotPath();
        for (int j = 0; j < another_path.size(); j++) {
            path.push_back(another_path[j]);
        }
    } else { //between any 2 arbitrary points and direction.
        double alpha = ((-a.theta + b.theta)) / 2;
        double cot_alpha;
        cot_alpha = cos(alpha) / sin(alpha);
        State center(0, 0, 0); //center of the circle--which is the locus of the point q between which 2 eulers are drawn.

        //cout << "alpha is " << alpha << endl;
        center.x = (a.x + b.x + cot_alpha * (a.y - b.y)) / 2;
        center.y = (a.y + b.y + cot_alpha * (b.x - a.x)) / 2;
        double r = sqrt(center.distance(a)); //radius of the circle.
        //cout << "centre " << center.x << " " << center.y << endl;
        double deflection1 = actualAtan(atan((center.y - a.y) / (center.x - a.x)), center, a);
        double deflection2 = actualAtan(atan((center.y - b.y) / (center.x - b.x)), center, b);

        //cout << "deflection between a and center" << deflection1 << "deflection between b and center" << deflection2 << endl;
        if (deflection2 < deflection1) {
            //swap
            double temp = deflection2;
            deflection2 = deflection1;
            deflection1 = temp;
        }
        double def = ((deflection2 - deflection1) / 2) + deflection1;

        State q(0, 0, 0);
        q.x = center.x + r * cos(def);
        q.y = center.y + r * sin(def);

        //cout << endl << endl << "q has diff in distance : " << (center.distance(a) - center.distance(q)) << endl;
        double theta1 = atan((q.y - a.y) / (q.x - a.x));
        double theta2 = atan((q.y - b.y) / (q.x - b.x));
        double beta1 = 2 * theta1 - a.theta;
        double beta2 = 2 * theta2 - b.theta;
        //cout << "path should be smooth enough " << beta1 - beta2 << endl;
        q.theta = beta1;
        //cout << "q is " << q.x << "," << q.y << ", " << q.theta << endl;
        curve.getControls(a, q);

        curve.getTrajectory();
        path = curve.plotPath();

        curve.getControls(q, b);

        curve.getTrajectory();

        std::vector<State> another_path = curve.plotPath();
        for (int j = 0; j < another_path.size(); j++) {
            path.push_back(another_path[j]);
        }
    }
    paths.poses.resize(path.size());
    for (int i = 0; i < path.size(); i++) {
        paths.poses[i].pose.position.x = path[i].x;
        paths.poses[i].pose.position.y = path[i].y;
        paths.poses[i].pose.position.z = path[i].theta;
    }

    return paths;
}

#endif

