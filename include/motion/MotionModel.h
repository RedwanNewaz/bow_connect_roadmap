//
// Created by redwan on 1/19/26.
//

#ifndef BOWCONNECT_MOTIONMODEL_H
#define BOWCONNECT_MOTIONMODEL_H
#include <vector>
#include <Eigen/Dense>
#include "../common/ParamManager.h"
#include <functional>

template<typename State, typename Control>
class MotionModel {
public:
    MotionModel(const ParamPtr& pm): pm_(pm) {
        max_speed_      = pm_->get_param<double>("max_speed");
        min_speed_      = pm_->get_param<double>("min_speed");
        max_yawrate_    = pm_->get_param<double>("max_yawrate");
    }
    virtual std::vector<State> calcTrajectory(State x, const Control& u, const Eigen::VectorXd& goal, double dt, double predict_time, double goal_radius) const = 0;
    virtual State motionODE(const State& x, const Control& u, double dt) const = 0;

    virtual Eigen::VectorXd scaledU(const Eigen::VectorXd& u) const {
        Eigen::Vector2d uu;
        uu(0) = min_speed_ + (max_speed_ - min_speed_) * u(0);
        // yaw angle is symmetric (there is no min_yaw rate)
        uu(1) = -max_yawrate_ + 2 * max_yawrate_ * u(1);
        return uu;
    }

protected:
    // ODE-based motion model using differential equations

    virtual State rungeKutta4Integration(
            const std::function<void(const State&, State&, double)>& derivatives,
            const State& initial_state,
            double t0,
            double dt
    )const = 0;

    ParamPtr pm_;
    double max_speed_;
    double min_speed_;
    double max_yawrate_;
};

namespace ugv {
    using State = Eigen::VectorXd;
    using Control = Eigen::Vector2d;
    using Point = Eigen::Vector2d;
    using Traj = std::vector<State>;
    const int state_dim = 5;
}

namespace uav {
    using State = Eigen::VectorXd;
    using Control = Eigen::Vector4d;  // Changed from Vector3d to Vector4d for quadrotor
    using Point = Eigen::Vector3d;
    using Traj = std::vector<State>;
    const int state_dim = 8;
}
namespace manipulator {
    using State = Eigen::VectorXd;
    using Control = Eigen::VectorXd;  // Changed from Vector3d to Vector4d for quadrotor
    using Point = Eigen::Vector3d;
    using Traj = std::vector<State>;
    const int state_dim = 9;
}


#endif //BOWCONNECT_MOTIONMODEL_H
