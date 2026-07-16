#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "common/ParamManager.h"
#include "planner/BOWConnect.h"
#include "planner/MotionTree.h"
#include "motion/UnicycleModel.h"

namespace py = pybind11;

class BOWConnectPlanner {
public:
    explicit BOWConnectPlanner(const std::string& config_yaml)
        : pm_(std::make_shared<param_manager>(config_yaml)),
          cc_(std::make_shared<occupancy::OccupancyMap>(pm_)),
          mm_(std::static_pointer_cast<MotionModel<ugv::State, ugv::Control>>(
                  std::make_shared<ugv::UnicycleModel>(pm_))),
          default_solver_time_(pm_->get_param<double>("solver_time")) {}

    Eigen::MatrixXd plan(const Eigen::VectorXd& start, const Eigen::Vector2d& goal,
                         double solver_time, bool optimize, bool verbose) {
        if (start.size() < 3)
            throw std::invalid_argument("start must have at least 3 elements: [x, y, theta]");

        ugv::State x0 = ugv::State::Zero(ugv::state_dim);
        const auto n = std::min<Eigen::Index>(start.size(), ugv::state_dim);
        x0.head(n) = start.head(n);

        const double budget = solver_time > 0.0 ? solver_time : default_solver_time_;

        auto planner = std::make_shared<bow::BOWConnect<
                ugv::State, ugv::Control, ugv::Point, ugv::Traj>>(x0, goal, cc_, pm_, mm_);

        std::pair<bool, ugv::Traj> result;
        {
            // solve() runs its own worker threads; release the GIL while it works
            py::gil_scoped_release release;
            result = planner->solve(budget, verbose);
            if (result.first && optimize) {
                auto origin = pm_->get_param<std::vector<double>>("origin");
                auto robot_radius = pm_->get_param<double>("robot_radius");
                result.second = bow::MotionTree<ugv::State>::optimize_traj(
                        result.second, origin[0], origin[1], robot_radius);
            }
        }

        if (!result.first)
            throw std::runtime_error("BOWConnect failed to find a solution within " +
                                     std::to_string(budget) + " s");

        Eigen::MatrixXd traj(static_cast<Eigen::Index>(result.second.size()), ugv::state_dim);
        for (Eigen::Index i = 0; i < traj.rows(); ++i)
            traj.row(i) = result.second[static_cast<size_t>(i)].transpose();
        return traj;
    }

private:
    ParamPtr pm_;
    CCPtr cc_;
    std::shared_ptr<MotionModel<ugv::State, ugv::Control>> mm_;
    double default_solver_time_;
};

PYBIND11_MODULE(bow_connect, m) {
    m.doc() = "Python bindings for the BOWConnect UGV motion planner";

    py::class_<BOWConnectPlanner>(m, "BOWConnectPlanner")
        .def(py::init<const std::string&>(), py::arg("config_yaml"),
             "Create a planner from an environment YAML config "
             "(map, robot, and solver parameters)")
        .def("plan", &BOWConnectPlanner::plan,
             py::arg("start"), py::arg("goal"),
             py::arg("solver_time") = -1.0,
             py::arg("optimize") = false,
             py::arg("verbose") = false,
             R"doc(Plan a trajectory from start to goal.

Args:
    start: [x, y, theta] (extra entries up to the 5-dim state are accepted)
    goal: [x, y]
    solver_time: planning budget in seconds; <= 0 uses `solver_time` from the config
    optimize: post-process the trajectory with MotionTree.optimize_traj
    verbose: print solver progress

Returns:
    numpy array of shape (N, 5) with rows [x, y, theta, v, omega]

Raises:
    RuntimeError: if no solution is found within the time budget
)doc");
}
