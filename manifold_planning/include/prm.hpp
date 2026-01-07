#pragma once
#include <vector>
#include <mutex>

struct PRMNode {
    std::vector<double> q;
    bool connected_to_goal = false;
};

class PRM {
public:
    void addSample(const std::vector<double>& q, bool goal_connected);
    bool isConnectedToGoal(const std::vector<double>& q) const;
    std::vector<PRMNode> getSamples() const;

private:
    std::vector<PRMNode> nodes_;
    mutable std::mutex mutex_;
};
