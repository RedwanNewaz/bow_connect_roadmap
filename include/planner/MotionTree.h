//
// Created by redwan on 1/20/26.
//

#ifndef MOTIONPLANNERBENCHMARK_MOTIONTREE_H
#define MOTIONPLANNERBENCHMARK_MOTIONTREE_H
#include "BOW.h"


namespace bow{
template<class State>
class MotionTree {
public:
    MotionTree(double origin_x, double origin_y, double grid_size);

    int addState(const State& state, int parent_idx);
    std::vector<State> getTrajectory(int goal_idx) const;
    int findNearestNode(const State& query) const;
    size_t getHash(const State& state) const;
    bool hasState(const State& state) const;
    const State& getState(int idx) const;
    size_t size() const { return nodes_.size(); }
    int getIndex(const State& state) const ;

    static std::vector<State> optimize_traj(const std::vector<State>& traj, double origin_x, double origin_y, double grid_size);


private:
    struct Node {
        State state;
        int parent_index;
    };

    std::vector<Node> nodes_;
    std::unordered_map<std::size_t, int> state_to_idx_;
    double origin_x_;
    double origin_y_;
    double grid_size_;

    std::size_t hashState(const State& s) const;
};


}

#endif //MOTIONPLANNERBENCHMARK_MOTIONTREE_H
