#ifndef __LOCAL_PLANNER__CLOTHOID__
#define __LOCAL_PLANNER__CLOTHOID__

static const double PI=3.14159;
static const int HEIGHT=800,WIDTH=800;
using namespace std;

#include <road_navigation/header.hpp>
#include <road_navigation/state.hpp>
#include <road_navigation/path.hpp>
#include <road_navigation/trajectory.hpp>

namespace navigation {

class Poses {
public:
    State a, b;
    
    Poses(State a_, State b_) {
        a = a_;
        b = b_;
    }
};

 class ClothoidPath : public PathSegment
    {
    public:
        double sigma;
        ClothoidPath(std::vector<State>& sites_,double sigma_,double larc_)
        {
            sites=sites_;
            sigma=sigma_;
            larc=larc_;
        }
    };
class Clothoid: public Trajectory{
    
   
public:
    State start,end;
    double x0,y0;
        double sigma,kMax,theta,theta_,k0,theta0;
    std::vector<State> path;
    cv::Mat img;
    Clothoid();
	int signum(double a);
	double calc_d(double alpha);
    double inRange(double theta);
    double actualAtan(double theta, State a, State b);
    int fresnel(double x, double &costerm, double &sinterm);
    void getXY(double s,double a,double b,double c,double& x,double& y);
    std::vector<PathSegment*> drawPath(geometry_msgs::Pose current_pose, geometry_msgs::Pose target_pose);
    void plotClothoid(State a,State b);
    void getControls(State a,State b);
    void getTrajectory();
    std::vector<PathSegment*> getPath(State a,State b);
    vector<State> plotPath();
};

}


#endif
