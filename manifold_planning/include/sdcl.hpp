#pragma once
#include "prm.hpp"
#include "svm_manifold.hpp"
#include "robot_interface.hpp"

class SDCL {
public:
    SDCL(RobotInterface* robot);

    void update(PRM& prm);
    std::vector<std::vector<double>> getManifoldSeeds() const;

private:
    RobotInterface* robot_;
    SVMManifold manifold_;
    std::vector<std::vector<double>> manifold_seeds_;
};
