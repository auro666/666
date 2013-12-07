#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <local_planner/MapUpdate.h>
#include <cstring>
#include <iostream>
#include <string>

using namespace std;

#include <sbpl/headers.h>

class FreeStyle {
private:
    EnvironmentNAVXYTHETALAT env;
    MDPConfig MDPCfg;
    ADPlanner* planner;
    int num_cols, num_rows;

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
        map_update.request.pose = pose;
        vector<nav2dcell_t> changes;

        if (client.call(map_update)) {
            if (map_update.response.valid) {
                unsigned char *snippet = map_update.response.snippet;
                int sensing_range = map_update.response.sense_range;
                for (int x = -sensing_range; x <= sensing_range; x++) {
                    for (int y = -sensing_range; y <= sensing_range; y++) {
                        int map_x = x + pose.position.x;
                        int map_y = y + pose.position.y;
                        if ((0 <= map_x) && (map_x < num_cols) && (0 <= map_y) && (map_y < num_rows)) {
                            int map_index = map_x * num_cols + map_y;
                            int snippet_index = (x + sensing_range) * (2 * sensing_range + 1) + (y + sensing_range);

                            if (env.GetMapCost(map_y, map_x) != snippet[snippet_index]) {
                                env.UpdateCost(map_y, map_x, snippet[snippet_index]);
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
            ROS_WARN("[fs_planner] : Server Call Failed");
        }

        return changes;
    }

    void publishPath(vector<sbpl_xy_theta_pt_t> sbpl_path) {

    }

public:

    FreeStyle(char *mprim_file) {
        //TODO: Fetch all these parameters from rosparam

        int width = 100; // Meters
        int height = 100; // Meters
        double start_x = 50; // Meters
        double start_y = 10; // Meters
        double start_theta = 0; // Radians
        double goal_x = 50; // Meters
        double goal_y = 90; // Meters
        double goal_theta = 0; // Radians
        double goal_tolerance_x = .001; // Meters
        double goal_tolerance_y = .001; // Meters
        double goal_tolerance_theta = .001; // Radians
        int num_thetas = 16;
        double cell_size = .025; // Meters
        double nominal_velocity = 1.0; // Meters per Second
        double time_to_turn_45_degs_inplace = 2.0; // Seconds
        unsigned char obstacle_threshold = 254;
        unsigned char cost_inscribed_threshold = 253;
        unsigned char cost_possibly_circumscribed_thresh = 128;

        num_cols = width / cell_size;
        num_rows = height / cell_size;
        vector<sbpl_2Dpt_t> perimeter = buildPerimeter();
        EnvNAVXYTHETALAT_InitParms params;
        params.numThetas = num_thetas;

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

        env.SetEnvParameter("cost_inscribed_thresh", cost_inscribed_threshold);
        env.SetEnvParameter("cost_possibly_circumscribed_thresh", cost_possibly_circumscribed_thresh);
        env.SetMap(new unsigned char[num_cols * num_rows]);
        env.SetGoal(goal_x, goal_y, goal_theta);
        env.SetGoalTolerance(goal_tolerance_x, goal_tolerance_y, goal_tolerance_theta);
        env.SetStart(start_x, start_y, start_theta);

        if (!env.InitializeMDPCfg(MDPCfg)) {
            throw new SBPL_Exception();
        }

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

    void plan_incremental(const geometry_msgs::Pose::ConstPtr& _pose) {
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
} fs_planner;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "fs_planner");
    ros::NodeHandle n;

    fs_planner = FreeStyle();

    ros::Subscriber state_sub = n.subscribe("localization/pose", 2, fs_planner.plan_incremental);

    return 0;
}
