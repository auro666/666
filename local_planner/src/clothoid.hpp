#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>


static const double PI=3.14159;
static const int HEIGHT=800,WIDTH=800;
using namespace std;

class State
{
public:
	double x,y,theta,curvature;
	State(double x_,double y_,double theta_,double curvature_);
    State(double x_,double y_,double theta_);
    State();
    double distance(State b);

};

class Clothoid{
public:
    State start,end;
    double x0,y0;
	double sigma,kMax,larc,theta,theta_,k0,theta0;
    std::vector<State> path;
    cv::Mat img;
    Clothoid();
	int signum(double a);
	double calc_d(double alpha);
    double inRange(double theta);
    int fresnel(double x, double &costerm, double &sinterm);
    void getXY(double s,double a,double b,double c,double& x,double& y);
    void plotClothoid(State a,State b);
    void getControls(State a,State b);
    void getTrajectory();
    vector<State> plotPath();
};

