
#ifndef __LOCAL_PLANNER__ROAD_NAVIGATION__
#define __LOCAL_PLANNER__ROAD_NAVIGATION__

#include <road_navigation/header.hpp>
#include <road_navigation/clothoid.hpp>

namespace navigation
{
	class RoadNavigation
	{
    public:
        // State initialState,targetState;
        // std::vector<State> targetTraj;
        // std::vector<Path*> pathCollection;
        // Trajectory* trajectory;
        nav_msgs::Path decideTargetTrajectory(nav_msgs::Path::ConstPtr lane_traj);
        double getDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) ;
        std::vector<geometry_msgs::Pose> getTargets(geometry_msgs::Pose current_pose, nav_msgs::Path target_traj) ;
        std::vector<std::vector<PathSegment*> > getPaths(geometry_msgs::Pose current_pose, std::vector<geometry_msgs::Pose> targets);
        void planRoadDetection(const nav_msgs::Path::ConstPtr& lane_traj, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
	};
}


#endif
