#include "sdcl.hpp"
#include <random>
#include <cmath>

SDCL::SDCL(RobotInterface* robot)
    : robot_(robot) {}

void SDCL::update(PRM& prm) {
    auto samples = prm.getSamples();
    if (samples.size() < 10) return;

    std::vector<std::vector<double>> X;
    std::vector<int> y;

    for (const auto& s : samples) {
        X.push_back(s.q);
        y.push_back(s.connected_to_goal ? 1 : -1);
    }

    manifold_.train(X, y);

    // Sample manifold points via projection
    manifold_seeds_.clear();
    for (const auto& s : samples) {
        double val = manifold_.evaluate(s.q);
        if (std::abs(val) < 0.1) {
            manifold_seeds_.push_back(s.q);
        }
    }
}

std::vector<std::vector<double>> SDCL::getManifoldSeeds() const {
    return manifold_seeds_;
}
