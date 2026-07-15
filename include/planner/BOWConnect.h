#pragma once

#include "bow_param.h"
#include "../common/ParamManager.h"
#include "../common/CollisionChecker.hh"
#include "../motion/MotionModel.h"
#include "BOW.h"
#include "Worker.h"
#include <vector>
#include <array>
#include <Eigen/Core>
#include <random>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

namespace bow {

// State trait system for generic state handling
template<typename Point>
struct StateTraits {
    // Default: Assume 2D point (x, y) and heading at index 2
    static constexpr size_t position_dim = 2;
    static constexpr size_t heading_index = 2;
    
    static Eigen::Vector2d getPosition(const Eigen::VectorXd& state) {
        return state.template head<2>();
    }
    
    static double getHeading(const Eigen::VectorXd& state) {
        if (state.size() > 2) return state(2);
        return 0.0;
    }
    
    static void setHeading(Eigen::VectorXd& state, double heading) {
        if (state.size() > 2) state(2) = heading;
    }
    
    static void setPosition(Eigen::VectorXd& state, const Eigen::Vector2d& pos) {
        if (state.size() >= 2) {
            state(0) = pos(0);
            state(1) = pos(1);
        }
    }
};

template<class State, class Control, class Point, class Traj>
class BOWConnect : public BOW<State, Control, Point, Traj> {
public:
    // Constructor
    BOWConnect(const State& x, const Point& goal, const CCPtr& cc, const ParamPtr& pm, const std::shared_ptr<MotionModel<State, Control>>& mm);

    // Destructor
    ~BOWConnect() = default;

    // Disable copy and move to prevent issues with potential threading
    BOWConnect(const BOWConnect&) = delete;
    BOWConnect& operator=(const BOWConnect&) = delete;
    BOWConnect(BOWConnect&&) = delete;
    BOWConnect& operator=(BOWConnect&&) = delete;

    // Main solve method (overrides BOPlanner::solve)
    std::pair<bool, Traj> solve(double time, bool verbose) override;

private:
    // Additional parameters specific to BOWConnect
    double max_dyawrate_;
    int num_threads_;
    int state_dim_;  // Store state dimension for generic initialization

    // Helper methods
    std::vector<State> sampleRandomState(const Point &center, double R, int N);

    // Boundary value problem solver for connecting trajectories
    std::pair<bool, Traj> solveBVP(const State& start, const State& goal, double max_time);

    // Check if two states can be connected respecting kinematic constraints
    // relax_constraints=true uses permissive checks suitable for tree connection finding
    bool isKinematicallyFeasible(const State& s1, const State& s2, bool relax_constraints = false) const;

    // Generate trajectory between two states using kinematic model
    Traj generateConnectingTrajectory(const State& start, const State& goal);

    // Smooth heading transitions in trajectory
    Traj smoothTrajectoryHeadings(const Traj& traj) const;

    // Solver status enum
    enum SolverStatus {
        RUNNING,
        FORWARD_SUCCESS,
        BACKWARD_SUCCESS,
        CONNECTED,
        TIMEOUT
    };
};

} // namespace bow
