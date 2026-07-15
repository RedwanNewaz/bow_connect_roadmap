//
// Created by redwan on 1/20/26.
//

#ifndef MOTIONPLANNERBENCHMARK_WORKER_H
#define MOTIONPLANNERBENCHMARK_WORKER_H

#include "BOW.h"
#include "MotionTree.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <cmath>

namespace bow{
template<class State, class Control, class Point, class Traj>
class WorkerThread {
public:
    WorkerThread(const ParamPtr& pm, const CCPtr& cc, const State& start_state, const Point& goal_point, double timeout)
        : timeout_(timeout)
        , solution_found_(false)
        , terminate_(false)
        , sync_counter_(0)
        , parent_index_(-1)
        , pm_(pm)  // Store the shared pointer
        , cc_(cc)  // Store the shared pointer
    {
        auto origin = pm_->get_param<std::vector<double>>("origin");
        double grid_size = pm_->get_param<double>("robot_radius");
        double origin_x = origin[0];
        double origin_y = origin[1];
        motion_tree_ = std::make_unique<MotionTree<State>>(origin_x, origin_y, grid_size);
        motion_tree_->addState(start_state, -1);
        // Create a MotionModel based on available parameters
        std::shared_ptr<MotionModel<State, Control>> mm;
        if(pm_->has_param("wheel_base"))
            mm = std::make_shared<ugv::BicycleModel>(pm_);
        else
            mm = std::make_shared<ugv::UnicycleModel>(pm_);
        // Use the stored member variables instead of calling getSharedPtr()
        planner_ = std::make_unique<BOW<State, Control, Point, Traj>>(start_state, goal_point, cc_, pm_, mm);
    }

    // Disable copy and move to prevent issues with mutex
    WorkerThread(const WorkerThread&) = delete;
    WorkerThread& operator=(const WorkerThread&) = delete;
    WorkerThread(WorkerThread&&) = delete;
    WorkerThread& operator=(WorkerThread&&) = delete;

    ~WorkerThread() {
        stop();
    }

    void start(){
        if (!thread_.joinable()) {
            thread_ = std::thread(&WorkerThread::run, this);
        }
    }

    void run(){
        bool found = false;
        Traj result;

        std::tie(found, result) = planner_->solve(timeout_, false);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            solution_found_ = found;
            final_result_ = std::move(result);
        }

        terminate_.store(true, std::memory_order_release);
    }

    void sync(){
        std::lock_guard<std::mutex> lock(mutex_);

        // Get the current trajectory from the BOW planner
        const Traj planner_result = planner_->getFinalResultCopy();

        for(size_t i = sync_counter_; i < planner_result.size(); i++){
            parent_index_ = motion_tree_->addState(planner_result[i], parent_index_);
            ++sync_counter_;
        }
    }

    void stop(){
        if(thread_.joinable()){
            thread_.join();
        }
    }

    void preemptPlanning() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (planner_) {
            planner_->preempt = true;
        }
    }

    void addState(const State& s){
        std::lock_guard<std::mutex> lock(mutex_);
        motion_tree_->addState(s, parent_index_);
    }

    Traj getPartialTraj(){
        std::lock_guard<std::mutex> lock(mutex_);
        // Return a copy to avoid data races
        return planner_->getFinalResultCopy();
    }

    // Get only the last N states to reduce memory copying
    Traj getRecentStates(size_t N = 50) {
        std::lock_guard<std::mutex> lock(mutex_);
        return planner_->getFinalResultRecent(N);
    }

    bool isSolutionFound() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return solution_found_;
    }

    bool connect(const State& s){
        std::lock_guard<std::mutex> lock(mutex_);
        return motion_tree_->hasState(s);
    }

    Traj get_trajectory(const State& s, int forward=1){
        std::lock_guard<std::mutex> lock(mutex_);

        int idx = motion_tree_->getIndex(s);
        auto traj = motion_tree_->getTrajectory(idx);

        if(forward < 0) {
            std::reverse(traj.begin(), traj.end());
            
            // When reversing trajectory, smooth the heading transitions
            // Use low-pass filter approach for smooth heading changes
            if (traj.size() > 2) {
                for (size_t i = 1; i < traj.size() - 1; ++i) {
                    if (traj[i].size() > 2) {
                        // Compute direction from previous to next state
                        Eigen::Vector2d direction = traj[i+1].template head<2>() - traj[i-1].template head<2>();
                        
                        if (direction.norm() > 1e-6) {
                            // Set heading based on the direction to next state
                            double target_heading = std::atan2(traj[i+1](1) - traj[i](1), 
                                                               traj[i+1](0) - traj[i](0));
                            
                            // Smooth transition from current heading to target
                            double curr_heading = traj[i](2);
                            double heading_diff = target_heading - curr_heading;
                            
                            // Normalize to [-pi, pi]
                            while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
                            while (heading_diff < -M_PI) heading_diff += 2 * M_PI;
                            
                            // Apply smoothing factor (0.7 means 70% toward target)
                            traj[i](2) = curr_heading + heading_diff * 0.7;
                        }
                    }
                }
            }
            
            // Fix first and last state headings
            if (traj.size() > 1 && traj[0].size() > 2) {
                traj[0](2) = std::atan2(traj[1](1) - traj[0](1), traj[1](0) - traj[0](0));
            }
            if (traj.size() > 1 && traj.back().size() > 2) {
                size_t last = traj.size() - 1;
                traj[last](2) = std::atan2(traj[last](1) - traj[last-1](1), 
                                         traj[last](0) - traj[last-1](0));
            }
        }

        return traj;
    }

    Traj getSolutionTraj(int forward=1) {
        std::lock_guard<std::mutex> lock(mutex_);

        auto traj = final_result_;

        if(forward < 0) {
            std::reverse(traj.begin(), traj.end());
            
            // Apply same heading smoothing as get_trajectory
            if (traj.size() > 2) {
                for (size_t i = 1; i < traj.size() - 1; ++i) {
                    if (traj[i].size() > 2) {
                        // Compute direction from previous to next state
                        Eigen::Vector2d direction = traj[i+1].template head<2>() - traj[i-1].template head<2>();
                        
                        if (direction.norm() > 1e-6) {
                            // Set heading based on the direction to next state
                            double target_heading = std::atan2(traj[i+1](1) - traj[i](1), 
                                                               traj[i+1](0) - traj[i](0));
                            
                            // Smooth transition from current heading to target
                            double curr_heading = traj[i](2);
                            double heading_diff = target_heading - curr_heading;
                            
                            // Normalize to [-pi, pi]
                            while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
                            while (heading_diff < -M_PI) heading_diff += 2 * M_PI;
                            
                            // Apply smoothing factor
                            traj[i](2) = curr_heading + heading_diff * 0.7;
                        }
                    }
                }
            }
            
            // Fix first and last state headings
            if (traj.size() > 1 && traj[0].size() > 2) {
                traj[0](2) = std::atan2(traj[1](1) - traj[0](1), traj[1](0) - traj[0](0));
            }
            if (traj.size() > 1 && traj.back().size() > 2) {
                size_t last = traj.size() - 1;
                traj[last](2) = std::atan2(traj[last](1) - traj[last-1](1), 
                                         traj[last](0) - traj[last-1](0));
            }
        }

        return traj;
    }

    bool isTerminated() const {
        return terminate_.load(std::memory_order_acquire);
    }

private:
    std::thread thread_;
    double timeout_;

    // Store shared pointers as member variables
    ParamPtr pm_;
    CCPtr cc_;

    // Protected by mutex_
    mutable std::mutex mutex_;
    bool solution_found_;
    Traj final_result_;
    std::unique_ptr<MotionTree<State>> motion_tree_;
    std::unique_ptr<BOW<State, Control, Point, Traj>> planner_;
    size_t sync_counter_;
    int parent_index_;

    // Atomic flag for termination
    std::atomic<bool> terminate_;
};

}
#endif //MOTIONPLANNERBENCHMARK_WORKER_H
