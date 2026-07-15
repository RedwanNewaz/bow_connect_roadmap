#include <iostream>
#include "common/ParamManager.h"
#include "planner/BOWConnect.h"


template <typename T>
void save_trajectory(const std::vector<T>& traj)
{
    // save trajectory in a csv file
    std::ofstream file("trajectory.csv");
    if (file.is_open()) {
        for (const auto& state : traj) {
            file << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << "," << state(4) << "\n";
        }
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
}
// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.

int main(int argc, char **argv) {
    assert(argc > 1 && "<env.yaml> arg is missing!");
    auto param_manager_ = std::make_shared<param_manager>(argv[1]);
    // Initialize collision checker (occupancy map)
    auto collision_checker_ = std::make_shared<occupancy::OccupancyMap>(param_manager_);
    auto motion_model_ = std::make_shared<ugv::UnicycleModel>(param_manager_);

    auto start = param_manager_->get_param<std::vector<double>>("start");
    auto goal   = param_manager_->get_param<std::vector<double>>("goal");
    auto solver_time = param_manager_->get_param<double>("solver_time");

    ugv::State robot_state;
    ugv::Point goal_position;

    robot_state.resize(ugv::state_dim);

    robot_state << start[0], start[1], start[2], 0.0, 0.0;
    goal_position << goal[0], goal[1];

    // Create BOWConnect planner
    // Cast motion model to base type using explicit types from ugv namespace
    auto mm_ptr = std::static_pointer_cast<MotionModel<ugv::State, ugv::Control>>(motion_model_);

    auto planner = std::make_shared<bow::BOWConnect<
        ugv::State, ugv::Control, ugv::Point, ugv::Traj>>(
        robot_state, goal_position, collision_checker_,
        param_manager_, mm_ptr);
    auto result = planner->solve(solver_time, true);

    if (argc != 2) {
        std::cout << "optimizing trajectory..." << std::endl;
        int n = result.second.size();
        auto robot_radius = param_manager_->get_param<double>("robot_radius");
        auto origin     = param_manager_->get_param<std::vector<double>>("origin");
        result.second = bow::MotionTree<ugv::State>::optimize_traj(result.second,
                                origin[0], origin[1], robot_radius);
        std::cout << "traj reduced from = " << n  << " to = " <<result.second.size() << std::endl;
    }



    if (result.first) {
        // extract trajectory
        save_trajectory(result.second);
    }
    return 0;
}