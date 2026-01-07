#pragma once
#include <vector>
#include "svm_manifold.hpp"
#include "prm.hpp"

/**
 * Algorithm 4 – Elastic triangulation update
 */
class ElasticUpdater {
public:
    ElasticUpdater(SVMManifold* manifold);

    /// Projects q to the learned manifold
    /// Returns empty vector if projection fails
    std::vector<double> project(
        const std::vector<double>& q);

private:
    SVMManifold* manifold_;
};
