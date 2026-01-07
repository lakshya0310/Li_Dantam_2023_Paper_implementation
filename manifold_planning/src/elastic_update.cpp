#include "elastic_update.hpp"
#include <nlopt.hpp>
#include <cmath>

ElasticUpdater::ElasticUpdater(SVMManifold* manifold)
    : manifold_(manifold) {}

std::vector<double> ElasticUpdater::project(
    const std::vector<double>& q) {

    if (!manifold_->trained()) return {};

    nlopt::opt opt(nlopt::LN_COBYLA, q.size());

    opt.set_min_objective(
        [](const std::vector<double>& x,
           std::vector<double>&,
           void* data) {
            return std::abs(
                static_cast<SVMManifold*>(data)->evaluate(x));
        },
        manifold_);

    opt.set_xtol_rel(1e-3);

    std::vector<double> x = q;
    try {
        opt.optimize(x);
    } catch (...) {
        return {};
    }
    return x;
}
