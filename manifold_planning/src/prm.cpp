#include "prm.hpp"

void PRM::addSample(const std::vector<double>& q, bool goal_connected) {
    std::lock_guard<std::mutex> lock(mutex_);
    nodes_.push_back({q, goal_connected});
}

bool PRM::isConnectedToGoal(const std::vector<double>& q) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& n : nodes_) {
        if (n.q == q)
            return n.connected_to_goal;
    }
    return false;
}

std::vector<PRMNode> PRM::getSamples() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return nodes_;
}
