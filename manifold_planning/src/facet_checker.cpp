#include "facet_checker.hpp"
#include "math_utils.hpp"

FacetChecker::FacetChecker(
    RobotInterface* robot,
    ElasticUpdater* updater)
    : robot_(robot), updater_(updater) {}

bool FacetChecker::smallEnough(
    const std::vector<std::vector<double>>& simplex) const {

    for (size_t i = 0; i < simplex.size(); ++i)
        for (size_t j = i + 1; j < simplex.size(); ++j)
            if (l2_norm(simplex[i]) - l2_norm(simplex[j]) > 0.02)
                return false;
    return true;
}

bool FacetChecker::allInCollision(
    const std::vector<std::vector<double>>& simplex) const {

    for (const auto& v : simplex)
        if (!robot_->inCollision(v))
            return false;
    return true;
}

bool FacetChecker::checkFacet(
    const std::vector<std::vector<double>>& simplex) {

    if (smallEnough(simplex))
        return allInCollision(simplex);

    size_t idx = longest_edge(simplex);
    size_t i = idx / simplex.size();
    size_t j = idx % simplex.size();

    auto mid = midpoint(simplex[i], simplex[j]);
    auto proj = updater_->project(mid);
    if (!proj.empty()) mid = proj;

    auto s1 = simplex;
    auto s2 = simplex;
    s1[i] = mid;
    s2[j] = mid;

    return checkFacet(s1) && checkFacet(s2);
}
