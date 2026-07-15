#pragma once
#include "bow_param.h"
#include "../common/ParamManager.h"
#include "../common/CollisionChecker.hh"
#include <vector>
#include <array>
#include <Eigen/Core>
#include <random>
#include <chrono>
#include <variant>
#include <mutex>
#include <atomic>

#include "../motion/UnicycleModel.h"
#include "../motion/BicycleModel.h"



namespace bow {
    static std::once_flag limbo_init_flag;

    inline double getMemoryUsageLocal() {
        std::ifstream file("/proc/self/statm");
        if (!file) return 0.0;
        long pages;
        file >> pages;
        return pages * sysconf(_SC_PAGESIZE) / (1024.0 * 1024.0);
    }

    template<class State, class Control, class Point, class Traj>
    class BOW {
    public:
        // Bayesian Optimization parameters
        BO_PARAM(size_t, dim_in, limbo::Params::num_control_dim);
        BO_PARAM(size_t, dim_out, 1);
        BO_PARAM(size_t, nb_constraints, 1);
        std::atomic<bool> preempt{false};
        Traj final_result;

        // Thread-safe accessors for final_result
        Traj getFinalResultCopy() const {
            std::lock_guard<std::mutex> lock(result_mutex_);
            return final_result;
        }

        Traj getFinalResultRecent(size_t n) const {
            std::lock_guard<std::mutex> lock(result_mutex_);
            if (final_result.size() <= n) {
                return final_result;
            }
            return Traj(final_result.end() - n, final_result.end());
        }

        // Constructor
        BOW(const State& x, const Point& goal, const CCPtr& cc, const ParamPtr& pm, std::shared_ptr<MotionModel<State, Control>> motion_model): goal_(goal)
            , x_(x)
            , cc_(cc)
            , pm_(pm)
            , motion_model_(motion_model)
        {

            max_speed_      = pm_->get_param<double>("max_speed");
            min_speed_      = pm_->get_param<double>("min_speed");
            max_yawrate_    = pm_->get_param<double>("max_yawrate");
            dt_             = pm_->get_param<double>("dt");
            goal_radius_    = pm_->get_param<double>("goal_radius");
            robot_radius_   = pm_->get_param<double>("robot_radius");
            predict_time_   = pm_->get_param<double>("predict_time");
            pref_speed_index_ = pm_->get_param<double>("pref_speed_index");
            limbo::Params::num_samples = pm_->get_param<double>("num_samples");

        }

        // compute optimal control for a finite planning horizon
        Eigen::VectorXd computeControl(){
            using namespace limbo;
            using Stop_t = boost::fusion::vector<stop::MaxIterations<Params>>;
            using Stat_t = boost::fusion::vector<
                    stat::Samples<Params>,
                    stat::BestObservations<Params>,
                    stat::AggregatedObservations<Params>
            >;
            //        using Mean_t = mean::Data<Params>;
            using Kernel_t = kernel::SquaredExpARD<Params>;
            using Mean_t = mean::Constant<Params>;
            //        using Kernel_t = kernel::Exp<Params>;
            using GP_t = model::GP<Params, Kernel_t, Mean_t>;
            using Constrained_GP_t = model::GP<Params, Kernel_t, Mean_t>;
            using Acqui_t = experimental::acqui::ECI<Params, GP_t, Constrained_GP_t>;
            using Init_t = init::RandomSampling<Params>;

            std::call_once(limbo_init_flag, [](){ 
                #ifdef USE_PARALLEL
                tools::par::init(); 
                #endif
            });

            experimental::bayes_opt::CBOptimizer<
                    Params,
                    modelfun<GP_t>,
                    acquifun<Acqui_t>,
                    statsfun<Stat_t>,
                    initfun<Init_t>,
                    stopcrit<Stop_t>,
                    experimental::constraint_modelfun<Constrained_GP_t>
            > opt;
            // std::cout << " [MEM] Before optimize: " << (int)getMemoryUsageLocal() << "MB" << std::endl;
            opt.optimize(*this);
            // std::cout << " [MEM] After optimize: " << (int)getMemoryUsageLocal() << "MB" << std::endl;
            auto uu = opt.best_sample();
            Eigen::VectorXd  uu_scaled = motion_model_->scaledU(uu);
            return  uu_scaled;
        }

        //  Public interface
        virtual std::pair<bool, Traj> solve(double time, bool verbose=true){

            auto terminate = [&](const State& x)
            {
                double dist_sq = (x.head(goal_.size()) - goal_).squaredNorm();
                return dist_sq < (goal_radius_ * goal_radius_);
            };


            // Excute planner and Record end time
            auto start_time = std::chrono::high_resolution_clock::now();

            bool solution_found = false;
            do{
                BOW mpc(x_, goal_, cc_->getSharedPtr(), pm_->getSharedPtr(), motion_model_);
                // std::cout << " [MEM] Before computeControl: " << (int)getMemoryUsageLocal() << "MB" << std::endl;
                auto u = mpc.computeControl();
                // std::cout << " [MEM] After computeControl: " << (int)getMemoryUsageLocal() << "MB" << std::endl;
                auto traj = calcTrajectory(x_, u,  goal_);
                if (traj.empty()) {
                    continue;
                }
                if (cc_->isCollision(traj))
                    continue;

                // instead of one step, we can use preferred speed
                int max_index = static_cast<int>(traj.size()) - 1;
                int N = std::min(max_index, pref_speed_index_);
                if (N < 0 || N >= static_cast<int>(traj.size())) {
                    continue;
                }

                const int copy_dim = std::min<int>(x_.size(), traj[N].size());
                if (copy_dim <= 0) {
                    continue;
                }

                for (int i = 0; i < copy_dim; i++)
                    x_[i] = traj[N](i);
                for (int i = copy_dim; i < x_.size(); i++)
                    x_[i] = 0.0;

                for (int i = 0; i <= N; ++i) {
                    State fixed = traj[i];
                    if (fixed.size() < x_.size()) {
                        const int old_size = fixed.size();
                        fixed.conservativeResize(x_.size());
                        for (int j = old_size; j < fixed.size(); ++j) {
                            fixed(j) = 0.0;
                        }
                    }
                    {
                        std::lock_guard<std::mutex> lock(result_mutex_);
                        final_result.push_back(std::move(fixed));
                    }
                }
                solution_found = terminate(x_);
                
                auto current_time = std::chrono::high_resolution_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count();
                if (elapsed > time) {
                    if (verbose) std::cout << "[BOW] Timeout reached." << std::endl;
                    break;
                }

            }  while (!solution_found && !preempt);

            // update current state using state transition function
            auto end_time = std::chrono::high_resolution_clock::now();
            // Calculate duration
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            if(verbose)
                std::cout << "[BOW]: Solution found in time: " << elapsed_time << " ms" << std::endl;

            Traj result_copy;
            {
                std::lock_guard<std::mutex> lock(result_mutex_);
                result_copy = final_result;
            }
            return std::make_pair(solution_found, std::move(result_copy));
        }

        // Operator for Bayesian optimization
        Eigen::VectorXd operator()(const Eigen::VectorXd& u) const{
            Eigen::VectorXd res(2);

            // limbo sample u in [0, 1] range which needs to map/scale to the robot's vel domains
            Eigen::VectorXd uu = motion_model_->scaledU(u);

            // feed forward trajectory for a sampled control for a given predict_time
            Traj traj = calcTrajectory(x_, uu,  goal_);
            auto[index, goalDist] = calcToGoalCost(traj, goal_);
            // erase the part of trajectory that does not help to reach goal location
            traj.erase(traj.begin() + index + 1, traj.end());
            // add safety constraint in terms of distance between robot and nearset obstacles
            res(1) = 1.0 - static_cast<float>(cc_->isCollision(traj));
            // convert the cost function to positive reward
            res(0) = (res(1) > 0.0 ) ? -goalDist : -1.0e3;
            return res;
        }

    protected:
        // Member variables - now protected for inheritance
        Point goal_;
        State x_;
        CCPtr cc_;
        ParamPtr pm_;

        // yaml config parameters
        double max_speed_;
        double min_speed_;
        double max_yawrate_;
        double dt_;
        double goal_radius_;
        double robot_radius_;
        double predict_time_;
        int pref_speed_index_;

        std::shared_ptr<MotionModel<State, Control>> motion_model_;

        // Protect final_result across threads
        mutable std::mutex result_mutex_;

        // Helper functions - now protected for inheritance
        // Eigen::Vector2d scaledU(const Eigen::VectorXd& u) const;

        // ODE-based motion model
        State motionODE(const State& x, const Control& u, double dt) const {
            return motion_model_->motionODE(x, u, dt);;
        }

        Traj calcTrajectory(State x, const Control& u, const Point& goal) const{
            return motion_model_->calcTrajectory(x, u, goal, dt_, predict_time_, goal_radius_);
        }


        std::pair<int, double> calcToGoalCost(const Traj& traj, const Point& goal) const{
            // Early exit for empty trajectory
            if (traj.empty()) {
                return {-1, std::numeric_limits<double>::max()};
            }

            double minDist = std::numeric_limits<double>::max();
            int bestIndex = -1;

            for (int i = 0; i < traj.size(); ++i) {
                double dist = (traj[i].head(goal.size()) - goal).norm();
                if (dist < minDist) {
                    minDist = dist;
                    bestIndex = i;
                }
            }

            return {bestIndex, minDist};
        }




    };

} // namespace bow
