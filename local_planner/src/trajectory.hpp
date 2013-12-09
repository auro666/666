
#ifndef __LOCAL_PLANNER__TRAJECTORY__
#define __LOCAL_PLANNER__TRAJECTORY__

#include <iostream>
#include <vector>
#include "path.hpp"
#include "state.hpp"
namespace navigation
{
	class Trajectory
	{
        public:
		double larc;
        virtual std::vector<PathSegment*> drawPath(geometry_msgs::Pose current_pose, geometry_msgs::Pose target_pose){
        }
	};
}

#endif