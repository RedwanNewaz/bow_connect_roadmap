//
// Created by airlab on 1/21/26.
//

//
// Standard kinodynamic bicycle model (car-like)
// State  x = [px, py, yaw, v, delta]^T
// Control u = [a, delta_rate]^T
//
// Created by redwan on 1/19/26. (adapted)
//

#ifndef BOWCONNECT_BICYCLEMODEL_H
#define BOWCONNECT_BICYCLEMODEL_H
#include "MotionModel.h"
#include <cmath>
#include <functional>


namespace ugv {

    class BicycleModel : public MotionModel<State, Control> {
    public:
        explicit BicycleModel(const ParamPtr& pm): MotionModel<State, Control>(pm)
        {
            L_ = pm_->get_param<double>("wheel_base");
            double max_accel = pm_->get_param<double>("max_accel");
            double delta_max = pm_->get_param<double>("max_yawrate");
            double delta_rate_max = pm_->get_param<double>("max_dyawrate");
            double max_speed      = pm_->get_param<double>("max_speed");
            double min_speed      = pm_->get_param<double>("min_speed");
            setLimits(min_speed, max_speed, max_accel, delta_max, delta_rate_max);

        }

        // Optional limits (set to <= 0 to disable)
        void setLimits(double v_min, double v_max,
                       double a_max,
                       double delta_max,
                       double delta_rate_max)
        {
            v_min_ = v_min; v_max_ = v_max;
            a_max_ = a_max;
            delta_max_ = delta_max;
            delta_rate_max_ = delta_rate_max;
        }

        std::vector<State> calcTrajectory(State x, const Control& u,
                                          const Eigen::VectorXd& goal,
                                          double dt, double predict_time,
                                          double goal_radius) const override
        {
            std::vector<State> traj;
            traj.reserve(static_cast<size_t>(predict_time / dt) + 1);

            Eigen::Vector2d goal_xy = goal.head<2>();
            traj.push_back(x);


            double time = 0.0;
            double dist = (goal_xy - x.head<2>()).norm();

//            std::cout << x.transpose() << std::endl;
//            std::cout << u.transpose() << std::endl;

            while (time <= predict_time && dist > goal_radius) {
                x = motionODE(x, u, dt);

                dist = (goal_xy - x.head<2>()).norm();
                traj.push_back(x);
                time += dt;
            }

            return traj;
        }

        State motionODE(const State& x, const Control& u, double dt) const override
        {
            // Apply control limits (optional)
            double a          = clampAbs(u(0), a_max_);
            double delta_rate = clampAbs(u(1), delta_rate_max_);

            // Integrate with RK4 using continuous derivatives
            auto deriv = [&](const State& s, State& dsdt, double) {
                const double px    = s(0);
                const double py    = s(1);
                const double yaw   = s(2);
                const double v     = s(3);
                const double delta = s(4);

                (void)px; (void)py; // silence unused warnings if any

                dsdt.setZero();
                dsdt(0) = v * std::cos(yaw);
                dsdt(1) = v * std::sin(yaw);
                dsdt(2) = (std::abs(std::cos(delta)) < 1e-9)
                              ? 0.0
                              : (v / L_) * std::tan(delta);
                dsdt(3) = a;
                dsdt(4) = delta_rate;
            };

            State next = rungeKutta4Integration(deriv, x, 0.0, dt);

            // Enforce state limits (optional)
            if (v_max_ > v_min_) {
                next(3) = clamp(next(3), v_min_, v_max_);
            }
            if (delta_max_ > 0.0) {
                next(4) = clamp(next(4), -delta_max_, delta_max_);
            }

            // Normalize yaw to [-pi, pi]
            next(2) = normalizeAngle(next(2));

            return next;
        }

    protected:
        State rungeKutta4Integration(
            const std::function<void(const State&, State&, double)>& derivatives,
            const State& initial_state,
            double t0,
            double dt) const override
        {
            int N = initial_state.size();
            State k1(N), k2(N), k3(N), k4(N);
            State s = initial_state;

            derivatives(s, k1, t0);           k1 *= dt;
            derivatives(s + 0.5 * k1, k2, t0 + 0.5 * dt); k2 *= dt;
            derivatives(s + 0.5 * k2, k3, t0 + 0.5 * dt); k3 *= dt;
            derivatives(s + k3,       k4, t0 + dt);       k4 *= dt;

            s += (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0;
            return s;
        }

    private:
        double L_;

        // limits (<=0 disables for those that use abs clamp)
        double v_min_ = -1e9;
        double v_max_ =  1e9;
        double a_max_ =  0.0;   // abs clamp if >0
        double delta_max_ = 0.0; // abs clamp if >0
        double delta_rate_max_ = 0.0; // abs clamp if >0

        static double clamp(double val, double lo, double hi) {
            return std::max(lo, std::min(val, hi));
        }

        static double clampAbs(double val, double abs_max) {
            if (abs_max <= 0.0) return val;
            return clamp(val, -abs_max, abs_max);
        }

        static double normalizeAngle(double a) {
            // [-pi, pi]
            a = std::fmod(a + M_PI, 2.0 * M_PI);
            if (a < 0) a += 2.0 * M_PI;
            return a - M_PI;
        }
    };

} // namespace ugv

#endif // BOWCONNECT_BICYCLEMODEL_H

