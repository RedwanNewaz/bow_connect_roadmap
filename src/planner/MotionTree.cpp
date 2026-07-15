//
// Created by redwan on 1/20/26.
//
#include "planner/MotionTree.h"

namespace bow{

template<class State>
MotionTree<State>::MotionTree(double origin_x, double origin_y, double grid_size)
        : origin_x_(origin_x), origin_y_(origin_y), grid_size_(grid_size) {}

template<class State>
int MotionTree<State>::addState(const State& state, int parent_idx) {
    size_t h = hashState(state);

    if (state_to_idx_.find(h) != state_to_idx_.end()) {
        return state_to_idx_[h];
    }

    Node newNode;
    newNode.state = state;
    newNode.parent_index = parent_idx;

    nodes_.push_back(newNode);
    int new_idx = static_cast<int>(nodes_.size() - 1);
    state_to_idx_[h] = new_idx;
    return new_idx;
}

template<class State>
std::vector<State> MotionTree<State>::getTrajectory(int goal_idx) const {
    std::vector<State> path;
    int curr = goal_idx;

    while (curr >= 0 && curr < nodes_.size()) {
        path.push_back(nodes_[curr].state);
        curr = nodes_[curr].parent_index;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

    template<class State>
int MotionTree<State>::findNearestNode(const State& query) const {
    if (nodes_.empty()) return -1;

    int best_idx = 0;
    double best_dist = (nodes_[0].state.template head<2>() - query.template head<2>()).squaredNorm();

    for (size_t i = 1; i < nodes_.size(); ++i) {
        double dist = (nodes_[i].state.template head<2>() - query.template head<2>()).squaredNorm();
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = static_cast<int>(i);
        }
    }
    return best_idx;
}

    template<class State>
size_t MotionTree<State>::getHash(const State& state) const {
    return hashState(state);
}

    template<class State>
bool MotionTree<State>::hasState(const State& state) const {
    size_t h = hashState(state);
    return state_to_idx_.find(h) != state_to_idx_.end();
}

    template<class State>
const State& MotionTree<State>::getState(int idx) const {
    return nodes_[idx].state;
}

    template<class State>
std::size_t MotionTree<State>::hashState(const State& s) const {
    std::uint16_t x = static_cast<std::uint16_t>(std::round((s(0) - origin_x_) / grid_size_));
    std::uint16_t y = static_cast<std::uint16_t>(std::round((s(1) - origin_y_) / grid_size_));
    return (static_cast<std::size_t>(x) << 16) | static_cast<std::size_t>(y);
}
    template<class State>
int MotionTree<State>::getIndex(const State &state) const{
    size_t h = hashState(state);
    auto it = state_to_idx_.find(h);
    if (it != state_to_idx_.end()) {
        return it->second;
    }
    return -1;
}

template<class State>
std::vector<State> MotionTree<State>::optimize_traj(
    const std::vector<State>& traj, double origin_x, double origin_y, double grid_size) {
    int last_added_idx = -1; // Root's parent is -1
    MotionTree<State> tree(origin_x, origin_y, grid_size);
    for (const auto& state : traj) {
        last_added_idx = tree.addState(state, last_added_idx);
    }

    return tree.getTrajectory(last_added_idx);
}

// Explicit template instantiation for UGV types
template class MotionTree<Eigen::VectorXd>;

}
