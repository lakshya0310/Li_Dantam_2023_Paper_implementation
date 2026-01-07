#pragma once
#include <vector>
#include "robot_interface.hpp"
#include "elastic_update.hpp"

/**
 * Recursive simplex bisection & checking (Section V-C)
 */
class FacetChecker {
public:
    FacetChecker(RobotInterface* robot,
                 ElasticUpdater* updater);

    bool checkFacet(
        const std::vector<std::vector<double>>& simplex);

private:
    RobotInterface* robot_;
    ElasticUpdater* updater_;

    bool smallEnough(
        const std::vector<std::vector<double>>& simplex) const;

    bool allInCollision(
        const std::vector<std::vector<double>>& simplex) const;
};
