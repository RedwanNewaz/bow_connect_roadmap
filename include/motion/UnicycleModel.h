//
// Created by redwan on 1/19/26.
//

#ifndef BOWCONNECT_UNICYCLEMODEL_H
#define BOWCONNECT_UNICYCLEMODEL_H
#include "MotionModel.h"

namespace ugv{

    class UnicycleModel : public MotionModel<State , Control> {
    public:
        explicit UnicycleModel(const ParamPtr& pm): MotionModel<State, Control>(pm)
        {

        }

        std::vector<State> calcTrajectory(State x, const Control& u, const Eigen::VectorXd &goal, double dt, double predict_time, double goal_radius) const override
        {
            // Pre-allocate maximum possible trajectory size to avoid reallocations
            std::vector<State>  traj;
            traj.reserve(static_cast<size_t>(predict_time / dt) + 1);

            // Use Eigen's aligned vector for better performance
            Eigen::Matrix<double, 2, 1, Eigen::DontAlign> goal_vec = goal;

            // Use Eigen's vector operations for distance calculation
            traj.push_back(x);

                  // Preallocate variables to avoid repeated memory allocations
            double time = 0.0;
            Eigen::Matrix<double, 2, 1, Eigen::DontAlign> pos_diff;

            // Use Eigen::NumTraits for better numerical stability
            double distance = Eigen::NumTraits<double>::highest();

            // Use vectorized operations and early termination
            while (time <= predict_time && distance > goal_radius) {
                // Optimize motion calculation
                x = motionODE(x, u, dt);

                // Vectorized distance calculation
                pos_diff = goal_vec - x.head<2>();
                distance = pos_diff.norm();

                traj.push_back(x);
                time += dt;
            }

            // Optional: Shrink to fit to release extra memory
            traj.shrink_to_fit();

            return traj;
        }

        State motionODE(const State & x, const Control & u, double dt) const override
        {
            // Define the state derivative function
            auto stateDeriv = [&](const State& state, State& dxdt, double) {
                // Kinematic bicycle model differential equations
                dxdt(0) = state(3) * std::cos(state(2)); // dx/dt = v * cos(theta)
                dxdt(1) = state(3) * std::sin(state(2)); // dy/dt = v * sin(theta)
                dxdt(2) = state(4); // dtheta/dt = omega (yaw rate)
                dxdt(3) = 0; // dv/dt = 0 (constant velocity control)
                dxdt(4) = 0; // domega/dt = 0 (constant yaw rate control)
            };

            // Use Eigen's RungeKutta4 integrator for numerical integration
            State next_state = rungeKutta4Integration(stateDeriv, x, 0, dt);

            // Normalize theta to [-pi, pi]
            next_state(2) = std::fmod(next_state(2) + M_PI, 2 * M_PI) - M_PI;

            // Directly set the control inputs
            next_state(3) = u(0);
            next_state(4) = u(1);

            return next_state;
        }

    protected:
        State rungeKutta4Integration(
                const std::function<void(const State &, State &, double)> &derivatives,
                const State &initial_state,
                double t0,
                double dt) const override
        {
            int N = initial_state.size();
            State k1(N), k2(N), k3(N), k4(N);
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
    };
}
#endif //BOWCONNECT_UNICYCLEMODEL_H
