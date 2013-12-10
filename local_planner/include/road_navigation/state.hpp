
#ifndef __LOCAL_PLANNER__STATE__
#define __LOCAL_PLANNER__STATE__

#include <iostream>

namespace navigation
{
	class State
    {
    public:
        double x,y,theta,curvature;
        State(double x_,double y_,double theta_,double curvature_);
        State(double x_,double y_,double theta_);
        State();
        double distance(State b);
        
    };
}

#endif