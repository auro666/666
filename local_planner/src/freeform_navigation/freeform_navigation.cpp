#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <local_planner/MapUpdate.h>
#include <cstring>
#include <iostream>
#include <string>

using namespace std;

#include <sbpl/headers.h>
#include <sbpl/utils/utils.h>

class FreeStyle {
private:
    EnvironmentNAVXYTHETALAT env;
    MDPConfig MDPCfg;
    ADPlanner* planner;
    int num_cols, num_rows;
    int count;
    double cell_size;

    ros::NodeHandle n;
    ros::Publisher path_pub;
    ros::Subscriber state_sub;
    ros::Subscriber goal_sub;

    vector<sbpl_2Dpt_t> buildPerimeter() {
        vector<sbpl_2Dpt_t> perimeter;
        sbpl_2Dpt_t point;
        double half_width = 0.01;
        double half_length = 0.01;
        point.x = -half_length;
        point.y = -half_width;
        perimeter.push_back(point);
        point.x = half_length;
        point.y = -half_width;
        perimeter.push_back(point);
        point.x = half_length;
        point.y = half_width;
        perimeter.push_back(point);
        point.x = -half_length;
        point.y = half_width;
        perimeter.push_back(point);

        return perimeter;
    }

    vector<nav2dcell_t> scanMapForChanges(geometry_msgs::Pose pose) {
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<local_planner::MapUpdate>("map_service");
        local_planner::MapUpdate map_update;
        map_update.request.pose.position.x = pose.position.x / cell_size;
        map_update.request.pose.position.y = pose.position.y / cell_size;
        map_update.request.pose.orientation = pose.orientation;
        vector<nav2dcell_t> changes;

        if (client.call(map_update)) {
            std::vector<unsigned char> snippet = map_update.response.snippet;
            std::vector<unsigned char> valid_flags = map_update.response.valid_flags;
            int sensing_range = map_update.response.sense_range;
            for (int x = -sensing_range; x <= sensing_range; x++) {
                for (int y = -sensing_range; y <= sensing_range; y++) {
                    int map_x = x + pose.position.x / cell_size;
                    int map_y = y + pose.position.y / cell_size;
                    if ((0 <= map_x) && (map_x < num_cols) && (0 <= map_y) && (map_y < num_rows)) {
                        int snippet_index = (x + sensing_range) + (y + sensing_range) * (2 * sensing_range + 1);
                        if (valid_flags.at(snippet_index)) {
                            if (env.GetMapCost(map_x, map_y) != snippet.at(snippet_index)) {
                                ROS_DEBUG(
                                        "[freeform_navigation] : Old: %d, New: %d",
                                        env.GetMapCost(map_x, map_y),
                                        snippet.at(snippet_index));

                                env.UpdateCost(map_x, map_y, snippet.at(snippet_index));
                                nav2dcell_t changed_cell;
                                changed_cell.x = map_x;
                                changed_cell.y = map_y;
                                changes.push_back(changed_cell);
                            }
                        }
                    }
                }
            }
        } else {
            ROS_WARN("[freeform_navigation] : Server Call Failed");
        }

        return changes;
    }

    void publishPath(vector<sbpl_xy_theta_pt_t> sbpl_path) {
        nav_msgs::Path path;
        path.poses.resize(sbpl_path.size());
        for (unsigned int i = 0; i < path.poses.size(); i++) {
            path.poses.at(i).pose.position.x = sbpl_path.at(i).x;
            path.poses.at(i).pose.position.y = sbpl_path.at(i).y;
            path.poses.at(i).pose.orientation = tf::createQuaternionMsgFromYaw(sbpl_path.at(i).theta);
        }
        path.header.seq = count;
        path.header.frame_id = count;
        path.header.stamp = ros::Time::now();
        path_pub.publish(path);
        count++;
    }

    void planIncremental(const geometry_msgs::Pose::ConstPtr& _pose) {
        // pose: meters
        planner->set_start(env.SetStart(
                _pose->position.x,
                _pose->position.y,
                tf::getYaw(_pose->orientation)));

        vector<nav2dcell_t> changed_cells = scanMapForChanges(*_pose);
        vector<int> predecessors;
        if (changed_cells.size() != 0) {
            env.GetPredsofChangedEdges(&changed_cells, &predecessors);
            planner->update_preds_of_changededges(&predecessors);
        }

        vector<int> solution_IDs;
        planner->replan(10, &solution_IDs);

        vector<sbpl_xy_theta_pt_t> sbpl_path;
        env.ConvertStateIDPathintoXYThetaPath(&solution_IDs, &sbpl_path);

        publishPath(sbpl_path);
    }

    void updateGoal(const geometry_msgs::Pose::ConstPtr& _pose) {
        planner->set_goal(env.SetGoal(
                _pose->position.x,
                _pose->position.y,
                tf::getYaw(_pose->orientation)));
    }

public:

    void initializeEnvironment(const char *mprim_file) {
        //TODO: Fetch all these parameters from rosparam

        int width = 10; // Meters
        int height = 10; // Meters
        double start_x = 5; // Meters
        double start_y = 1; // Meters
        double start_theta = 0; // Radians
        double goal_x = 5; // Meters
        double goal_y = 9; // Meters
        double goal_theta = 0; // Radians
        double goal_tolerance_x = .001; // Meters
        double goal_tolerance_y = .001; // Meters
        double goal_tolerance_theta = .001; // Radians
        int num_thetas = 16;
        cell_size = .025; // Meters
        double nominal_velocity = 1.0; // Meters per Second
        double time_to_turn_45_degs_inplace = 2.0; // Seconds
        unsigned char obstacle_threshold = 254;
        unsigned char cost_inscribed_threshold = 253;
        unsigned char cost_possibly_circumscribed_thresh = 128;

        num_cols = width / cell_size;
        num_rows = height / cell_size;
        unsigned char *map_data = new unsigned char[num_cols * num_rows];
        for (int i = 0; i < num_cols * num_rows; i++) {
            map_data[i] = 0;
        }

        vector<sbpl_2Dpt_t> perimeter = buildPerimeter();
        EnvNAVXYTHETALAT_InitParms params;
        params.startx = start_x;
        params.starty = start_y;
        params.starttheta = start_theta;
        params.mapdata = map_data;
        params.numThetas = num_thetas;

        env.SetEnvParameter("cost_inscribed_thresh", cost_inscribed_threshold);
        env.SetEnvParameter("cost_possibly_circumscribed_thresh", cost_possibly_circumscribed_thresh);

        if (!env.InitializeEnv(
                num_cols,
                num_rows,
                perimeter,
                cell_size,
                nominal_velocity,
                time_to_turn_45_degs_inplace,
                obstacle_threshold,
                mprim_file,
                params)) {
            throw new SBPL_Exception();
        }

        env.SetGoal(goal_x, goal_y, goal_theta);
        env.SetGoalTolerance(goal_tolerance_x, goal_tolerance_y, goal_tolerance_theta);

        if (!env.InitializeMDPCfg(&MDPCfg)) {
            throw new SBPL_Exception();
        }
    }

    void initializePlanner() {
        planner = new ADPlanner(&env, false); // FALSE -> BACKWARD Search

        if (planner->set_start(MDPCfg.startstateid) == 0) {
            throw new SBPL_Exception();
        }
        if (planner->set_goal(MDPCfg.goalstateid) == 0) {
            throw new SBPL_Exception();
        }
        planner->set_initialsolution_eps(3.0);
        planner->set_search_mode(false);
    }

    void setupComms() {
        count = 0;

        state_sub = n.subscribe("localization/pose", 2, &FreeStyle::planIncremental, this);
        goal_sub = n.subscribe("master_planner/next_way_point", 2, &FreeStyle::updateGoal, this);
        path_pub = n.advertise<nav_msgs::Path>("local_planner/path", 10);
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "freeform_navigation");

    // TODO: Create a master_planner node which would sequentially initiate all other nodes with data 

    //FreeStyle fs_planner = FreeStyle("../config/pr2.mprim");
    FreeStyle fs_planner = FreeStyle();
    fs_planner.initializeEnvironment("/home/samuel/fuerte_workspace/sandbox/666/local_planner/config/pr2.mprim");
    fs_planner.initializePlanner();
    fs_planner.setupComms();

    ros::spin();

    return 0;
}
