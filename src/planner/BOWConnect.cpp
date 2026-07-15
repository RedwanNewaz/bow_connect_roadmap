#include "planner/BOWConnect.h"

#include <cmath>
#include <algorithm>
#include <limits>
#include <functional>
#include <chrono>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <boost/fusion/include/vector.hpp>

#include <limbo/tools/macros.hpp>
#include <limbo/tools/parallel.hpp>
#include <limbo/experimental/bayes_opt/cboptimizer.hpp>

int limbo::Params::num_samples = 15;
int limbo::Params::num_control_dim = 2;


namespace bow {

    template<class State, class Control, class Point, class Traj>
    BOWConnect<State, Control, Point, Traj>::BOWConnect(const State& x, const Point& goal, const CCPtr& cc, const ParamPtr& pm, const std::shared_ptr<MotionModel<State, Control>>& mm)
            : BOW<State, Control, Point, Traj>(x, goal, cc, pm, mm)  // Call base class constructor
            , max_dyawrate_(0.0)
            , state_dim_(x.size())  // Capture state dimension from initial state
    {
        // Initialize BOWConnect-specific parameters
        max_dyawrate_ = this->pm_->template get_param<double>("max_dyawrate");
        if(this->pm_->has_param("num_threads"))
            num_threads_ = this->pm_->template get_param<int>("num_threads");
        else
            num_threads_ = std::thread::hardware_concurrency();

        std::cout << "[BOWConnect] planner will use " << num_threads_ << " threads (state_dim=" << state_dim_ << ").\n";
    }

    template<class State, class Control, class Point, class Traj>
    Traj BOWConnect<State, Control, Point, Traj>::smoothTrajectoryHeadings(const Traj& traj) const {
        if (traj.size() < 3) {
            return traj;
        }

        Traj smoothed_traj = traj;
        
        // Helper function to normalize angles to [-pi, pi]
        auto normalize_angle = [](double a) {
            while (a > M_PI) a -= 2 * M_PI;
            while (a < -M_PI) a += 2 * M_PI;
            return a;
        };
        
        // First pass: aggressive smoothing for ALL sharp transitions > 30 degrees  
        for (size_t i = 1; i < smoothed_traj.size(); ++i) {
            if (smoothed_traj[i].size() <= 2) continue;
            
            double prev_heading = smoothed_traj[i-1].size() > 2 ? smoothed_traj[i-1](2) : smoothed_traj[i](2);
            double curr_heading = smoothed_traj[i](2);
            double diff = normalize_angle(curr_heading - prev_heading);
            
            // If ANY sharp change (> 30 degrees), smooth it aggressively
            if (std::abs(diff) > M_PI * 0.166) {  // 30 degrees
                Eigen::Vector2d pos_diff = smoothed_traj[i].template head<2>() - smoothed_traj[i-1].template head<2>();
                double pos_distance = pos_diff.norm();
                
                if (pos_distance > 1e-6) {  // Valid position difference
                    // Smooth toward halfway point
                    double new_heading = prev_heading + normalize_angle(diff) * 0.3;
                    smoothed_traj[i](2) = new_heading;
                }
            }
        }
        
        // Second pass: cascade smoothing - propagate smooth headings forward
        for (size_t i = 2; i < smoothed_traj.size(); ++i) {
            if (smoothed_traj[i].size() <= 2) continue;
            
            double prev_heading = smoothed_traj[i-1].size() > 2 ? smoothed_traj[i-1](2) : smoothed_traj[i](2);
            double curr_heading = smoothed_traj[i](2);
            double prev_prev_heading = smoothed_traj[i-2].size() > 2 ? smoothed_traj[i-2](2) : smoothed_traj[i](2);
            
            double diff1 = normalize_angle(prev_heading - prev_prev_heading);
            double diff2 = normalize_angle(curr_heading - prev_heading);
            
            // If transitions are inconsistent, interpolate
            if (std::abs(diff1) < 0.1 && std::abs(diff2) > M_PI * 0.166) {
                smoothed_traj[i](2) = prev_heading + normalize_angle(diff2) * 0.4;
            }
        }
        
        return smoothed_traj;
    }

    template<class State, class Control, class Point, class Traj>
    bool BOWConnect<State, Control, Point, Traj>::isKinematicallyFeasible(const State& s1, const State& s2, bool relax_constraints) const {
        // Calculate position difference
        Eigen::Vector2d pos_diff = s2.template head<2>() - s1.template head<2>();
        double distance = pos_diff.norm();

        // Check if distance is too small
        if (distance < 1e-6) {
            return true;
        }

        // For connection attempts, use relaxed constraints
        if (relax_constraints) {
            // Just check if states are reachable (very permissive)
            double min_time_for_distance = distance / this->max_speed_;
            return min_time_for_distance < 200.0;  // Very generous time bound
        }

        // Calculate required heading to reach s2 from s1
        double required_heading = std::atan2(pos_diff(1), pos_diff(0));

        // Calculate heading difference (normalized to [-pi, pi])
        double heading_diff = std::fmod(required_heading - s1(2) + M_PI, 2 * M_PI) - M_PI;
        if (heading_diff > M_PI) heading_diff -= 2 * M_PI;
        if (heading_diff < -M_PI) heading_diff += 2 * M_PI;

        // Check if the required heading change is reasonable
        // A sharp angle would require more than 90 degrees of turn
        double max_allowed_heading_diff = M_PI * 0.5; // 90 degrees
        if (std::abs(heading_diff) > max_allowed_heading_diff) {
            return false;  // Too sharp of a turn
        }

        // Check if heading at s2 is compatible
        if (s2.size() > 2) {
            double heading_at_s2 = s2(2);
            double heading_diff_at_goal = std::fmod(heading_at_s2 - required_heading + M_PI, 2 * M_PI) - M_PI;
            if (heading_diff_at_goal > M_PI) heading_diff_at_goal -= 2 * M_PI;
            if (heading_diff_at_goal < -M_PI) heading_diff_at_goal += 2 * M_PI;
            
            // Allow some heading mismatch at goal but not too much
            if (std::abs(heading_diff_at_goal) > M_PI * 0.3) {  // 54 degrees
                return false;
            }
        }

        // Check if feasible with kinematic constraints
        double min_time_for_heading = std::abs(heading_diff) / this->max_yawrate_;
        double min_time_for_distance = distance / this->max_speed_;

        // Both constraints should be satisfiable (heading change and distance coverage)
        return (min_time_for_heading < 100.0 && min_time_for_distance < 100.0);
    }

    template<class State, class Control, class Point, class Traj>
    Traj BOWConnect<State, Control, Point, Traj>::generateConnectingTrajectory(const State& start, const State& goal) {
        Traj trajectory;

        // Calculate position and heading differences
        Eigen::Vector2d pos_diff = goal.template head<2>() - start.template head<2>();
        double distance = pos_diff.norm();

        if (distance < 1e-6) {
            trajectory.push_back(start);
            trajectory.push_back(goal);
            return trajectory;
        }

        // Calculate required heading to reach goal
        double required_heading = std::atan2(pos_diff(1), pos_diff(0));
        
        State current = start;
        trajectory.push_back(current);

        // Generate smooth trajectory using simple kinematic model
        // Gradually adjust heading while moving toward goal
        double time_step = this->dt_;
        double max_steps = 1000;  // Prevent infinite loops
        int steps = 0;

        while ((current.template head<2>() - goal.template head<2>()).norm() > this->robot_radius_ && steps < max_steps) {
            // Calculate current heading and desired heading
            Eigen::Vector2d current_pos_diff = goal.template head<2>() - current.template head<2>();
            double current_distance = current_pos_diff.norm();
            
            if (current_distance < this->robot_radius_) {
                break;
            }

            double desired_heading = std::atan2(current_pos_diff(1), current_pos_diff(0));
            double heading_error = desired_heading - current(2);
            
            // Normalize heading error to [-pi, pi]
            while (heading_error > M_PI) heading_error -= 2 * M_PI;
            while (heading_error < -M_PI) heading_error += 2 * M_PI;

            // Smooth heading correction with limited yaw rate
            double max_yaw_per_step = this->max_yawrate_ * time_step;
            double yawrate = std::clamp(heading_error / time_step, -(this->max_yawrate_), this->max_yawrate_);
            
            // Use proportional control for heading with damping
            double heading_correction = std::clamp(heading_error * 0.5, -max_yaw_per_step, max_yaw_per_step);
            yawrate = heading_correction / time_step;

            // Speed is proportional to remaining distance
            double speed = std::min(this->max_speed_, current_distance / time_step);
            speed = std::max(this->min_speed_, speed);

            Control u(speed, yawrate);
            current = this->motionODE(current, u, time_step);
            trajectory.push_back(current);
            
            steps++;
        }

        // Add final goal state
        trajectory.push_back(goal);

        return trajectory;
    }

    template<class State, class Control, class Point, class Traj>
    std::pair<bool, Traj> BOWConnect<State, Control, Point, Traj>::solveBVP(const State& start, const State& goal, double max_time) {
        // Check if connection is kinematically feasible (use relaxed constraints)
        if (!isKinematicallyFeasible(start, goal, true)) {
            std::cerr << "[BOWConnect-BVP]: Connection failed kinematic feasibility check" << std::endl;
            return {false, {}};
        }

        // Generate connecting trajectory
        Traj connecting_traj = generateConnectingTrajectory(start, goal);
        
        if (connecting_traj.empty()) {
            std::cerr << "[BOWConnect-BVP]: Generated trajectory is empty" << std::endl;
            return {false, {}};
        }

        // Check if trajectory is collision-free (use inherited cc_)
        if (this->cc_ && this->cc_->isCollision(connecting_traj)) {
            // std::cerr << "[BOWConnect-BVP]: Trajectory collision detected" << std::endl;
            return {false, {}};
        }

        // Verify final state is reasonably close to goal (relaxed check for connections)
        if (!connecting_traj.empty()) {
            double final_distance = (connecting_traj.back().template head<2>() - goal.template head<2>()).norm();
            // For BVP connections, allow larger final distance (since we're connecting trees)
            double max_connection_distance = std::max(this->goal_radius_ * 2.0, 2.0);
            if (final_distance > max_connection_distance) {
                std::cerr << "[BOWConnect-BVP]: Final distance " << final_distance 
                         << " exceeds threshold " << max_connection_distance << std::endl;
                return {false, {}};
            }
        }

        return {true, std::move(connecting_traj)};
    }

    template<class State, class Control, class Point, class Traj>
    std::pair<bool, Traj> BOWConnect<State, Control, Point, Traj>::solve(double time, bool verbose)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Sample start and goal states (use inherited x_ and goal_)
        std::vector<State> start_states_local;
        std::vector<State> goal_states_local;

        try {
            start_states_local = sampleRandomState(this->x_.template head<2>(), this->goal_radius_, num_threads_);
            goal_states_local = sampleRandomState(this->goal_, this->goal_radius_, num_threads_);
        } catch (const std::exception& e) {
            std::cerr << "[BOWConnect]: Failed to sample states: " << e.what() << std::endl;
            return {false, {}};
        }

        // Validate sample counts
        if (start_states_local.size() < 2 || goal_states_local.size() < 2) {
            std::cerr << "[BOWConnect]: Insufficient samples generated" << std::endl;
            return {false, {}};
        }

        // Create worker threads (use inherited pm_ and cc_)
        std::vector<std::unique_ptr<WorkerThread<State, Control, Point, Traj>>> forward_planners;
        std::vector<std::unique_ptr<WorkerThread<State, Control, Point, Traj>>> backward_planners;

        try {
            auto sample_size = std::min(start_states_local.size(), goal_states_local.size());
            for(size_t i = 0; i < sample_size; i += 2) {
                forward_planners.emplace_back(std::make_unique<WorkerThread<State, Control, Point, Traj>>(
                    this->pm_, this->cc_, start_states_local[i], goal_states_local[i].template head<2>(), time));

                backward_planners.emplace_back(std::make_unique<WorkerThread<State, Control, Point, Traj>>(
                    this->pm_, this->cc_, goal_states_local[i + 1], start_states_local[i + 1].template head<2>(), time));
            }
        } catch (const std::exception& e) {
            std::cerr << "[BOWConnect]: Failed to create planners: " << e.what() << std::endl;
            return {false, {}};
        }

        // Start all threads
        for(auto& planner : forward_planners) {
            if (planner) {
                planner->start();
            }
        }

        for(auto& planner : backward_planners) {
            if (planner) {
                planner->start();
            }
        }

        SolverStatus solver_status = RUNNING;
        size_t forward_solution_idx = 0;
        size_t backward_solution_idx = 0;
        size_t connected_forward_idx = 0;
        size_t connected_backward_idx = 0;
        State connection_state;
        
        // Track if we found forward/backward solutions but keep searching for better connected solutions
        bool found_forward_fallback = false;
        bool found_backward_fallback = false;

        // Main planning loop
        while(solver_status == RUNNING) {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed_sec = std::chrono::duration_cast<std::chrono::duration<double>>(
                current_time - start_time).count();

            if (elapsed_sec >= time) {
                solver_status = TIMEOUT;

                // Signal all threads to stop planning immediately
                for(auto& fp : forward_planners) {
                    if (fp) fp->preemptPlanning();
                }
                for(auto& bp : backward_planners) {
                    if (bp) bp->preemptPlanning();
                }
                break;
            }
            // Check forward planners
            for(size_t i = 0; i < forward_planners.size(); ++i) {
                auto& forward_planner = forward_planners[i];
                if (!forward_planner) continue;

                forward_planner->sync();

                if(forward_planner->isSolutionFound() && !found_forward_fallback) {
                    std::cout << "[BOWConnect]: Forward planner found solution (potential fallback)" << std::endl;
                    forward_solution_idx = i;
                    found_forward_fallback = true;
                    // Don't stop - keep searching for connected solutions
                }
            }

            // Only check backward planners if no forward solution found
            if (solver_status == RUNNING) {
                for(size_t i = 0; i < backward_planners.size(); ++i) {
                    auto& backward_planner = backward_planners[i];
                    if (!backward_planner) continue;

                    backward_planner->sync();

                    if(backward_planner->isSolutionFound() && !found_backward_fallback) {
                        std::cout << "[BOWConnect]: Backward planner found solution (potential fallback)" << std::endl;
                        backward_solution_idx = i;
                        found_backward_fallback = true;
                        // Don't stop - keep searching for connected solutions
                    }
                }
            }

            // Check for connections between forward and backward trees
            if (solver_status == RUNNING) {
                std::vector<std::tuple<double, size_t, size_t>> candidate_connections;
                
                // Collect all viable connections without immediately committing
                for(size_t f_idx = 0; f_idx < forward_planners.size(); ++f_idx) {
                    auto& forward_planner = forward_planners[f_idx];
                    if (!forward_planner) continue;

                    // Get only recent states to minimize memory copying
                    {
                        Traj forward_partial_path = forward_planner->getRecentStates(50);
                        if (forward_partial_path.empty()) {
                            continue;
                        }

                        for(const State& forward_state : forward_partial_path) {
                            for(size_t b_idx = 0; b_idx < backward_planners.size(); ++b_idx) {
                                auto& backward_planner = backward_planners[b_idx];
                                if (!backward_planner) continue;

                                if(backward_planner->connect(forward_state)) {
                                    // Found a connection - extract backward state
                                    Traj backward_traj = backward_planner->getRecentStates(50);
                                    if (backward_traj.empty()) {
                                        continue;
                                    }

                                    // Find closest backward state
                                    double min_dist = std::numeric_limits<double>::max();
                                    State closest_backward_state = backward_traj[0];

                                    for (const auto& backward_state : backward_traj) {
                                        double dist = (backward_state.template head<2>() - forward_state.template head<2>()).norm();
                                        if (dist < min_dist) {
                                            min_dist = dist;
                                            closest_backward_state = backward_state;
                                        }
                                    }

                                    // Check feasibility (use relaxed constraints for connection finding)
                                    if (isKinematicallyFeasible(forward_state, closest_backward_state, true)) {
                                        // Try to verify this connection can be solved before collecting
                                        auto [bvp_test, _] = solveBVP(forward_state, closest_backward_state, time);
                                        
                                        if (bvp_test) {
                                            // Connection is viable - but only accept if reasonably close
                                            // Distant connections are fragile and may fail when states change
                                            if (min_dist < 12.0) {
                                                // Connection is viable and close enough, add to candidates
                                                candidate_connections.emplace_back(min_dist, f_idx, b_idx);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                
                // Use the best (closest) connection if any exist
                if (!candidate_connections.empty()) {
                    // Sort by distance to find closest connection
                    std::sort(candidate_connections.begin(), candidate_connections.end(),
                        [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });
                    
                    auto [distance, f_idx, b_idx] = candidate_connections[0];
                    
                    solver_status = CONNECTED;
                    connected_forward_idx = f_idx;
                    connected_backward_idx = b_idx;
                    
                    std::cout << "[BOWConnect]: CONNECTED (verified via BVP, distance=" << distance << ")" << std::endl;
                    
                    // Signal all threads to stop planning immediately
                    for(auto& fp : forward_planners) {
                        if (fp) fp->preemptPlanning();
                    }
                    for(auto& bp : backward_planners) {
                        if (bp) bp->preemptPlanning();
                    }
                }
            }

            // If all workers terminated and no connection found, do final check before exiting
            if (solver_status == RUNNING) {
                bool all_terminated = true;
                for (const auto& fp : forward_planners) {
                    if (fp && !fp->isTerminated()) {
                        all_terminated = false;
                        break;
                    }
                }
                if (all_terminated) {
                    for (const auto& bp : backward_planners) {
                        if (bp && !bp->isTerminated()) {
                            all_terminated = false;
                            break;
                        }
                    }
                }

                if (all_terminated) {
                    // All workers have finished - do final synchronization and check for any solutions
                    if (verbose) {
                        std::cout << "[BOWConnect]: All workers terminated, performing final solution check..." << std::endl;
                    }
                    
                    // Final sync to ensure we have all states and solutions
                    for(size_t i = 0; i < forward_planners.size(); ++i) {
                        auto& fp = forward_planners[i];
                        if (fp) {
                            fp->sync();
                            if (fp->isSolutionFound() && !found_forward_fallback) {
                                forward_solution_idx = i;
                                found_forward_fallback = true;
                                if (verbose) {
                                    std::cout << "[BOWConnect]: Found forward solution in final check (planner " << i << ")" << std::endl;
                                }
                            }
                        }
                    }
                    
                    for(size_t i = 0; i < backward_planners.size(); ++i) {
                        auto& bp = backward_planners[i];
                        if (bp) {
                            bp->sync();
                            if (bp->isSolutionFound() && !found_backward_fallback) {
                                backward_solution_idx = i;
                                found_backward_fallback = true;
                                if (verbose) {
                                    std::cout << "[BOWConnect]: Found backward solution in final check (planner " << i << ")" << std::endl;
                                }
                            }
                        }
                    }
                    
                    // If we found any fallback solutions, update status accordingly
                    if (found_forward_fallback) {
                        solver_status = FORWARD_SUCCESS;
                        if (verbose) {
                            std::cout << "[BOWConnect]: Using forward fallback solution" << std::endl;
                        }
                    } else if (found_backward_fallback) {
                        solver_status = BACKWARD_SUCCESS;
                        if (verbose) {
                            std::cout << "[BOWConnect]: Using backward fallback solution" << std::endl;
                        }
                    } else {
                        solver_status = TIMEOUT;
                        if (verbose) {
                            std::cout << "[BOWConnect]: No solutions found in final check" << std::endl;
                        }
                    }
                    break;
                }
            }

            // Small sleep to prevent busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Stop all threads gracefully
        for(auto& forward_planner : forward_planners) {
            if (forward_planner) {
                forward_planner->stop();
            }
        }

        for(auto& backward_planner : backward_planners) {
            if (backward_planner) {
                backward_planner->stop();
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();

        if(verbose) {
            std::cout << "[BOWConnect]: Solution found in time: "
                      << elapsed_time << " ms" << std::endl;
        }

        // Extract and return solution based on status
        switch (solver_status) {
            case FORWARD_SUCCESS:
            {
                if (forward_solution_idx < forward_planners.size() &&
                    forward_planners[forward_solution_idx]) {
                    auto traj = forward_planners[forward_solution_idx]->getSolutionTraj();
                    
                    // Verify trajectory actually reaches the true goal
                    if (!traj.empty()) {
                        double final_distance = (traj.back().template head<2>() - this->goal_).norm();
                        if (verbose) {
                            std::cout << "[BOWConnect]: Forward trajectory distance to goal: " 
                                      << final_distance << " (threshold: " << this->goal_radius_ << ")" << std::endl;
                        }
                        
                        if (final_distance <= this->goal_radius_) {
                            if (verbose) {
                                std::cout << "[BOWConnect]: Returning forward trajectory with "
                                          << traj.size() << " states" << std::endl;
                            }
                            return {true, std::move(traj)};
                        } else {
                            if (verbose) {
                                std::cout << "[BOWConnect]: Forward trajectory does NOT reach goal, continuing search..." << std::endl;
                            }
                            // Don't return, continue to check other cases or wait for better solution
                        }
                    }
                }
                break;
            }

            case BACKWARD_SUCCESS:
            {
                if (backward_solution_idx < backward_planners.size() &&
                    backward_planners[backward_solution_idx]) {
                    auto traj = backward_planners[backward_solution_idx]->getSolutionTraj(-1);
                    
                    // Verify trajectory actually reaches the true goal (start point in reversed traj)
                    if (!traj.empty()) {
                        double final_distance = (traj.back().template head<2>() - this->goal_).norm();
                        if (verbose) {
                            std::cout << "[BOWConnect]: Backward trajectory distance to goal: " 
                                      << final_distance << " (threshold: " << this->goal_radius_ << ")" << std::endl;
                        }
                        
                        if (final_distance <= this->goal_radius_) {
                            if (verbose) {
                                std::cout << "[BOWConnect]: Returning backward trajectory with "
                                          << traj.size() << " states" << std::endl;
                            }
                            return {true, std::move(traj)};
                        } else {
                            if (verbose) {
                                std::cout << "[BOWConnect]: Backward trajectory does NOT reach goal, continuing search..." << std::endl;
                            }
                            // Don't return, continue to check other cases or wait for better solution
                        }
                    }
                }
                break;
            }

            case CONNECTED:
            {
                // Only merge trajectories from the planners that actually connected
                if (connected_forward_idx >= forward_planners.size() ||
                    !forward_planners[connected_forward_idx] ||
                    connected_backward_idx >= backward_planners.size() ||
                    !backward_planners[connected_backward_idx]) {
                    std::cerr << "[BOWConnect]: Invalid planner indices for connection" << std::endl;
                    break;
                }

                // Re-find the connection point from current tree states (avoid stale state copies)
                State forward_connection_state;
                State backward_connection_state;
                
                {
                    Traj forward_recent = forward_planners[connected_forward_idx]->getRecentStates(50);
                    Traj backward_recent = backward_planners[connected_backward_idx]->getRecentStates(50);
                    
                    if (forward_recent.empty() || backward_recent.empty()) {
                        std::cerr << "[BOWConnect]: Cannot find recent states at connection time" << std::endl;
                        break;
                    }
                    
                    // Find closest pair
                    double min_dist = std::numeric_limits<double>::max();
                    for (const auto& f_state : forward_recent) {
                        for (const auto& b_state : backward_recent) {
                            double dist = (f_state.template head<2>() - b_state.template head<2>()).norm();
                            if (dist < min_dist) {
                                min_dist = dist;
                                forward_connection_state = f_state;
                                backward_connection_state = b_state;
                            }
                        }
                    }
                }

                // Get the forward trajectory (partial path up to now)
                Traj f_traj;
                try {
                    f_traj = forward_planners[connected_forward_idx]->getPartialTraj();
                } catch (const std::exception& e) {
                    std::cerr << "[BOWConnect]: Error getting forward trajectory for merge: " << e.what() << std::endl;
                    break;
                }

                if (f_traj.empty()) {
                    std::cerr << "[BOWConnect]: Forward trajectory is empty at connection" << std::endl;
                    break;
                }

                // Truncate forward trajectory at connection point
                auto forward_it = std::find_if(f_traj.begin(), f_traj.end(),
                    [&](const State& s) {
                        return (s.template head<2>() - forward_connection_state.template head<2>()).norm() < 1e-3;
                    });

                if (forward_it != f_traj.end()) {
                    f_traj.erase(forward_it + 1, f_traj.end());
                }

                // Solve BVP to connect forward endpoint to backward tree
                auto [bvp_success, connecting_traj] = solveBVP(
                    forward_connection_state,
                    backward_connection_state,
                    time
                );

                // If BVP fails, try interpolation fallback
                if (!bvp_success) {
                    double state_distance = (backward_connection_state.template head<2>() - forward_connection_state.template head<2>()).norm();
                    
                    std::cerr << "[BOWConnect]: BVP failed in CONNECTED case (distance=" << state_distance << "), trying interpolation..." << std::endl;
                    
                    // Allow up to 15 units for interpolation fallback
                    if (state_distance < 15.0) {
                        // Simple linear interpolation with smaller step size
                        double step_size = 0.3;  // Interpolate every 0.3 units for safety
                        int num_interp_steps = std::max(2, static_cast<int>(state_distance / step_size));
                        connecting_traj.clear();
                        
                        for (int step = 0; step <= num_interp_steps; ++step) {
                            double alpha = static_cast<double>(step) / num_interp_steps;
                            State interp_state = forward_connection_state * (1.0 - alpha) + backward_connection_state * alpha;
                            connecting_traj.push_back(interp_state);
                        }
                        
                        // Check if interpolated trajectory is collision-free
                        if (this->cc_ && this->cc_->isCollision(connecting_traj)) {
                            std::cerr << "[BOWConnect]: Interpolated trajectory has collision, trying coarser interpolation..." << std::endl;
                            
                            // Try coarser interpolation with larger steps
                            connecting_traj.clear();
                            double coarse_step_size = 1.0;  // Much larger steps
                            int coarse_steps = std::max(2, static_cast<int>(state_distance / coarse_step_size));
                            
                            for (int step = 0; step <= coarse_steps; ++step) {
                                double alpha = static_cast<double>(step) / coarse_steps;
                                State interp_state = forward_connection_state * (1.0 - alpha) + backward_connection_state * alpha;
                                connecting_traj.push_back(interp_state);
                            }
                            
                            // Check again
                            if (this->cc_ && this->cc_->isCollision(connecting_traj)) {
                                std::cerr << "[BOWConnect]: Coarse interpolation also has collision, using direct connection" << std::endl;
                                // Last resort: just use the two endpoints
                                connecting_traj.clear();
                                connecting_traj.push_back(forward_connection_state);
                                connecting_traj.push_back(backward_connection_state);
                                
                                // Check if even this is valid
                                if (this->cc_ && this->cc_->isCollision(connecting_traj)) {
                                    std::cerr << "[BOWConnect]: Even direct endpoint connection has collision" << std::endl;
                                    break;
                                }
                            }
                        }
                        
                        std::cout << "[BOWConnect]: Fallback succeeded (" << connecting_traj.size() << " states)" << std::endl;
                        bvp_success = true;
                    } else {
                        std::cerr << "[BOWConnect]: States too far apart for fallback (distance=" << state_distance << ")" << std::endl;
                        break;
                    }
                }

                if (!bvp_success || connecting_traj.empty()) {
                    std::cerr << "[BOWConnect]: Failed to create connecting trajectory" << std::endl;
                    break;
                }

                // Get the backward trajectory from the connection point
                Traj b_traj;
                try {
                    b_traj = backward_planners[connected_backward_idx]->get_trajectory(backward_connection_state, -1);
                } catch (const std::exception& e) {
                    std::cerr << "[BOWConnect]: Error getting backward trajectory for merge: " << e.what() << std::endl;
                    break;
                }

                // If backward trajectory is empty or doesn't reach goal, try to extend it
                if (b_traj.empty() || (b_traj.back().template head<2>() - this->goal_).norm() > this->goal_radius_) {
                    if (verbose) {
                        if (b_traj.empty()) {
                            std::cout << "[BOWConnect]: Backward trajectory empty, attempting to connect to goal..." << std::endl;
                        } else {
                            std::cout << "[BOWConnect]: Backward trajectory doesn't reach goal (dist=" 
                                      << (b_traj.back().template head<2>() - this->goal_).norm() 
                                      << "), attempting to extend..." << std::endl;
                        }
                    }
                    
                    // Try to get trajectory from backward planner's start (which should be near goal)
                    Traj backward_full = backward_planners[connected_backward_idx]->getPartialTraj();
                    if (!backward_full.empty()) {
                        // The backward planner starts near goal, so its first state should be close to goal
                        if ((backward_full.front().template head<2>() - this->goal_).norm() <= this->goal_radius_) {
                            // Reverse the full backward trajectory to go from connection to goal
                            std::reverse(backward_full.begin(), backward_full.end());
                            
                            // Find where the connection state is in this trajectory
                            auto b_it = std::find_if(backward_full.begin(), backward_full.end(),
                                [&](const State& s) {
                                    return (s.template head<2>() - backward_connection_state.template head<2>()).norm() < 1e-3;
                                });
                            
                            if (b_it != backward_full.end()) {
                                // Take everything from connection point onwards (toward goal)
                                b_traj.assign(b_it, backward_full.end());
                                if (verbose) {
                                    std::cout << "[BOWConnect]: Extended backward trajectory to " << b_traj.size() << " states" << std::endl;
                                }
                            }
                        }
                    }
                    
                    // If still empty or doesn't reach goal, try direct BVP connection to goal
                    if (b_traj.empty() || (b_traj.back().template head<2>() - this->goal_).norm() > this->goal_radius_) {
                        State goal_state = this->x_;  // Use state dimension from x_
                        goal_state.template head<2>() = this->goal_;
                        
                        // Set heading toward goal if we have backward connection state
                        if (b_traj.empty() && backward_connection_state.size() >= 3) {
                            goal_state = backward_connection_state;
                            goal_state.template head<2>() = this->goal_;
                        } else if (!b_traj.empty() && b_traj.back().size() >= 3) {
                            goal_state = b_traj.back();
                            goal_state.template head<2>() = this->goal_;
                        }
                        
                        State start_for_goal_bvp = b_traj.empty() ? backward_connection_state : b_traj.back();
                        auto [goal_bvp_success, goal_connecting] = solveBVP(start_for_goal_bvp, goal_state, time);
                        
                        if (goal_bvp_success && !goal_connecting.empty()) {
                            b_traj.insert(b_traj.end(), goal_connecting.begin(), goal_connecting.end());
                            if (verbose) {
                                std::cout << "[BOWConnect]: Extended to goal via BVP (" << goal_connecting.size() << " states)" << std::endl;
                            }
                        } else if (verbose) {
                            std::cout << "[BOWConnect]: Could not extend backward trajectory to goal" << std::endl;
                        }
                    }
                }

                if (verbose) {
                    std::cout << "[BOWConnect]: Merging trajectories:" << std::endl;
                    std::cout << "  Forward: " << f_traj.size() << " states" << std::endl;
                    std::cout << "  BVP connecting: " << connecting_traj.size() << " states" << std::endl;
                    std::cout << "  Backward: " << b_traj.size() << " states" << std::endl;
                }

                // Validate trajectories are not empty
                if (f_traj.empty() && connecting_traj.empty() && b_traj.empty()) {
                    std::cerr << "[BOWConnect]: All trajectories are empty!" << std::endl;
                    break;
                }

                // Remove duplicate states at trajectory junctions to avoid sharp angles
                // Remove last state of f_traj if it's too close to first state of connecting_traj
                if (!f_traj.empty() && !connecting_traj.empty()) {
                    if ((f_traj.back().template head<2>() - connecting_traj.front().template head<2>()).norm() < 1e-3) {
                        f_traj.pop_back();
                    }
                }

                // Remove last state of connecting_traj if it's too close to first state of b_traj
                if (!connecting_traj.empty() && !b_traj.empty()) {
                    if ((connecting_traj.back().template head<2>() - b_traj.front().template head<2>()).norm() < 1e-3) {
                        connecting_traj.pop_back();
                    }
                }

                // Merge: forward + BVP connection + backward
                f_traj.insert(f_traj.end(), connecting_traj.begin(), connecting_traj.end());
                f_traj.insert(f_traj.end(), b_traj.begin(), b_traj.end());

                if (verbose) {
                    std::cout << "[BOWConnect]: Final merged trajectory: "
                              << f_traj.size() << " states" << std::endl;
                }
                
                // Verify merged trajectory reaches the true goal
                if (!f_traj.empty()) {
                    double final_distance = (f_traj.back().template head<2>() - this->goal_).norm();
                    if (verbose) {
                        std::cout << "[BOWConnect]: Merged trajectory distance to goal: " 
                                  << final_distance << " (threshold: " << this->goal_radius_ << ")" << std::endl;
                    }
                    
                    if (final_distance <= this->goal_radius_) {
                        if (verbose) {
                            std::cout << "[BOWConnect]: Connected solution reaches goal!" << std::endl;
                        }
                        return {true, std::move(f_traj)};
                    } else {
                        if (verbose) {
                            std::cout << "[BOWConnect]: Connected solution does NOT reach goal" << std::endl;
                        }
                        // Don't return - let planning continue if time permits
                    }
                }
                break;
            }

            case RUNNING:
            case TIMEOUT:
            default:
                if (verbose) {
                    std::cout << "[BOWConnect]: Main loop ended in RUNNING/TIMEOUT state" << std::endl;
                }

                // Re-scan for any completed solutions before returning (avoid missing late results)
                if (!found_forward_fallback) {
                    for (size_t i = 0; i < forward_planners.size(); ++i) {
                        if (forward_planners[i] && forward_planners[i]->isSolutionFound()) {
                            forward_solution_idx = i;
                            found_forward_fallback = true;
                            break;
                        }
                    }
                }

                if (!found_backward_fallback) {
                    for (size_t i = 0; i < backward_planners.size(); ++i) {
                        if (backward_planners[i] && backward_planners[i]->isSolutionFound()) {
                            backward_solution_idx = i;
                            found_backward_fallback = true;
                            break;
                        }
                    }
                }
                
                // Try fallback solutions if available
                if (found_forward_fallback && forward_solution_idx < forward_planners.size() && 
                    forward_planners[forward_solution_idx]) {
                    auto traj = forward_planners[forward_solution_idx]->getSolutionTraj();
                    if (!traj.empty()) {
                        double final_distance = (traj.back().template head<2>() - this->goal_).norm();
                        if (verbose) {
                            std::cout << "[BOWConnect]: Checking forward fallback trajectory, distance to goal: " 
                                      << final_distance << std::endl;
                        }
                        if (final_distance <= this->goal_radius_) {
                            if (verbose) {
                                std::cout << "[BOWConnect]: Using forward fallback solution with " 
                                          << traj.size() << " states" << std::endl;
                            }
                            return {true, std::move(traj)};
                        }
                    }
                }
                
                if (found_backward_fallback && backward_solution_idx < backward_planners.size() && 
                    backward_planners[backward_solution_idx]) {
                    auto traj = backward_planners[backward_solution_idx]->getSolutionTraj(-1);
                    if (!traj.empty()) {
                        double final_distance = (traj.back().template head<2>() - this->goal_).norm();
                        if (verbose) {
                            std::cout << "[BOWConnect]: Checking backward fallback trajectory, distance to goal: " 
                                      << final_distance << std::endl;
                        }
                        if (final_distance <= this->goal_radius_) {
                            if (verbose) {
                                std::cout << "[BOWConnect]: Using backward fallback solution with " 
                                          << traj.size() << " states" << std::endl;
                            }
                            return {true, std::move(traj)};
                        }
                    }
                }
                
                if (verbose) {
                    std::cout << "[BOWConnect]: No valid solution found" << std::endl;
                }
                break;
        }

        return {false, {}};
    }

    template<class State, class Control, class Point, class Traj>
    std::vector<State> BOWConnect<State, Control, Point, Traj>::sampleRandomState(const Point &center, double R, int N) {
        if (N <= 0) {
            throw std::invalid_argument("[BOWConnect]: Number of samples must be positive");
        }
        if (R <= 0.0) {
            throw std::invalid_argument("[BOWConnect]: Sampling radius must be positive");
        }

        std::vector<State> points;
        points.reserve(N);

        // Thread-local random number generation
        thread_local std::mt19937 gen(
            std::random_device{}() +
            std::hash<std::thread::id>{}(std::this_thread::get_id())
        );

        std::uniform_real_distribution<double> dist(0.0, 1.0);
        std::uniform_real_distribution<double> heading(-M_PI, M_PI);

        const int max_iterations = 1000;
        int iteration = 0;

        do {
            points.clear();
            points.reserve(N);

            for (int i = 0; i < N; ++i) {
                double theta = 2.0 * M_PI * dist(gen);
                double radius = std::sqrt(dist(gen)) * R;

                State state(state_dim_);  // Use stored state dimension
                
                // Generic position assignment using traits
                // Assumes Point has direct component access [0], [1], etc.
                Eigen::Vector2d center_2d;
                if constexpr (std::is_same_v<Point, Eigen::Vector2d>) {
                    center_2d = center;
                } else if constexpr (std::is_same_v<Point, Eigen::Vector3d>) {
                    center_2d << center(0), center(1);
                } else {
                    // Fallback for generic Point types
                    center_2d << center(0), center(1);
                }
                
                state(0) = center_2d(0) + radius * std::cos(theta);
                state(1) = center_2d(1) + radius * std::sin(theta);
                
                // Set heading at index 2 if state has at least 3 dimensions
                if (state_dim_ > 2) {
                    state(2) = heading(gen);
                }
                
                // Initialize remaining state components to zero
                // This makes the class work with UAV (8D), Manipulator (9D), etc.
                for (int j = 3; j < state_dim_; ++j) {
                    state(j) = 0.0;
                }

                points.push_back(std::move(state));
            }

            ++iteration;
            if (iteration >= max_iterations) {
                throw std::runtime_error(
                    "[BOWConnect]: Failed to sample collision-free states after "
                    + std::to_string(max_iterations) + " iterations");
            }

        } while (this->cc_ && this->cc_->isCollision(points));  // Use inherited cc_

        return points;
    }

    // Explicit template instantiations
    // UGV instantiation
    template class BOWConnect<Eigen::VectorXd, Eigen::Vector2d, Eigen::Vector2d, std::vector<Eigen::VectorXd>>;

} // namespace bow
