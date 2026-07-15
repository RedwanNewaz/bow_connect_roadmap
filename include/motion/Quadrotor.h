//
// Created by airlab on 1/21/26.
//

#ifndef QUADROTOR_H
#define QUADROTOR_H
#include "MotionModel.h"
#include "../common/ParamManager.h"

namespace uav {
class Quadrotor: public MotionModel<State, Control> {
public:
    Quadrotor(const ParamPtr& pm): MotionModel(pm) {

        state_dim_       = pm_->get_param<int>("state_dim");

        max_velocity_x_  = pm_->get_param<double>("max_velocity_x");
        min_velocity_x_  = pm_->get_param<double>("min_velocity_x");
        max_velocity_y_  = pm_->get_param<double>("max_velocity_y");
        min_velocity_y_  = pm_->get_param<double>("min_velocity_y");
        max_velocity_z_  = pm_->get_param<double>("max_velocity_z");
        min_velocity_z_  = pm_->get_param<double>("min_velocity_z");
        max_angular_vel_ = pm_->get_param<double>("max_angular_vel");
    }

    std::vector<State> calcTrajectory(State x, const Control& u, const Eigen::VectorXd& goal, double dt, double predict_time, double goal_radius) const override{
        // Pre-allocate maximum possible trajectory size to avoid reallocations
        Traj traj;
        traj.reserve(static_cast<size_t>(predict_time / dt) + 1);

        // Use Eigen's aligned vector for better performance
        Eigen::Matrix<double, 3, 1, Eigen::DontAlign> goal_vec = goal;

        // Initialize trajectory with current state
        traj.push_back(x);


        // Preallocate variables
        double time = 0.0;
        Eigen::Matrix<double, 3, 1, Eigen::DontAlign> pos_diff;
        double distance = Eigen::NumTraits<double>::highest();

        // Simulate trajectory until time limit or goal reached
        while (time <= predict_time && distance > goal_radius) {
            // Update state with dynamic model
            x = motionODE(x, u, dt);

            // Calculate distance to goal (only using x,y,z components)
            pos_diff = goal_vec - x.head<3>();
            distance = pos_diff.norm();

            traj.push_back(x);
            time += dt;
        }

        // Optional: Shrink to fit to release extra memory
        traj.shrink_to_fit();

        return traj;
    }


    Eigen::VectorXd scaledU(const Eigen::VectorXd& u) const override{
        Eigen::Vector4d uu;
        uu(0) = min_velocity_x_ + (max_velocity_x_ - min_velocity_x_) * u(0);
        uu(1) = min_velocity_y_ + (max_velocity_y_ - min_velocity_y_) * u(1);
        uu(2) = min_velocity_z_ + (max_velocity_z_ - min_velocity_z_) * u(2);
        uu(3) = -max_angular_vel_ + 2 * max_angular_vel_ * u(3);
        return uu;
    }


    State motionODE(const State& x, const Control& u, double dt) const override{
        // Define the state derivative function for quadrotor with velocity control
        auto stateDeriv = [&](const State& state, State& dxdt, double) {
            // Quadrotor state: [x, y, z, theta, x_dot, y_dot, z_dot, theta_dot]
            // Control input: [x_dot_cmd, y_dot_cmd, z_dot_cmd, theta_dot_cmd]

            // Position derivatives (from current velocities)
            dxdt(0) = state(4); // dx/dt = x_dot
            dxdt(1) = state(5); // dy/dt = y_dot
            dxdt(2) = state(6); // dz/dt = z_dot
            dxdt(3) = state(7); // dtheta/dt = theta_dot

            // Velocity dynamics (simplified model with time constants)
            const double tau_x = 0.5; // Time constant for x velocity
            const double tau_y = 0.5; // Time constant for y velocity
            const double tau_z = 0.4; // Time constant for z velocity
            const double tau_theta = 0.3; // Time constant for angular velocity

            // Velocity control with first-order dynamics
            dxdt(4) = (u(0) - state(4)) / tau_x; // dx_dot/dt = (x_dot_cmd - x_dot) / tau_x
            dxdt(5) = (u(1) - state(5)) / tau_y; // dy_dot/dt = (y_dot_cmd - y_dot) / tau_y
            dxdt(6) = (u(2) - state(6)) / tau_z; // dz_dot/dt = (z_dot_cmd - z_dot) / tau_z
            dxdt(7) = (u(3) - state(7)) / tau_theta; // dtheta_dot/dt = (theta_dot_cmd - theta_dot) / tau_theta
        };

        // Use RK4 integrator for numerical integration
        State next_state = rungeKutta4Integration(stateDeriv, x, 0, dt);

        // Normalize angle to [-pi, pi]
        next_state(3) = std::fmod(next_state(3) + M_PI, 2 * M_PI) - M_PI;

        return next_state;
    }

protected:
    State rungeKutta4Integration(
           const std::function<void(const State&, State&, double)>& derivatives,
           const State& initial_state,
           double t0,
           double dt
   ) const override{
        State k1, k2, k3, k4;
        k1.resize(state_dim_);
        k2.resize(state_dim_);
        k3.resize(state_dim_);
        k4.resize(state_dim_);

        State state = initial_state;

        // k1
        derivatives(state, k1, t0);
        k1 *= dt;

        // k2
        State temp_state = state + 0.5 * k1;
        derivatives(temp_state, k2, t0 + 0.5 * dt);
        k2 *= dt;

        // k3
        temp_state = state + 0.5 * k2;
        derivatives(temp_state, k3, t0 + 0.5 * dt);
        k3 *= dt;

        // k4
        temp_state = state + k3;
        derivatives(temp_state, k4, t0 + dt);
        k4 *= dt;

        // Final state update
        state += (k1 + 2*k2 + 2*k3 + k4) / 6.0;

        return state;
    }

private:
    int state_dim_;
    double max_velocity_x_;
    double min_velocity_x_;
    double max_velocity_y_;
    double min_velocity_y_;
    double max_velocity_z_;
    double min_velocity_z_;
    double max_angular_vel_;


};
}
#endif //QUADROTOR_H
