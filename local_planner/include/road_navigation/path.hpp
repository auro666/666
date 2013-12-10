
#ifndef __LOCAL_PLANNER__PATH_SEGMENTS__
#define __LOCAL_PLANNER__PATH_SEGMENTS__
#include <iostream>
#include <vector>
#include <road_navigation/state.hpp>
namespace navigation
{
	class PathSegment
	{
    public:
        double larc;
        std::vector<State> sites;
	};
}

#endif
