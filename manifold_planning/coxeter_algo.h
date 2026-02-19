#pragma once
#include <vector>
#include <functional>
#include <Eigen/Dense>

// Represents a Facet of the triangulation (a set of vertices on the manifold)
using Facet = std::vector<Eigen::VectorXd>;

/**
 * Generates the manifold triangulation.
 * * @param dimension Dimension of the C-space.
 * @param f The manifold function (SVM decision function).
 * @param seeds List of seed points (one for each disconnected component).
 * @param scale The triangulation size parameter (lambda_T).
 * @return A vector of Facets approximating the manifold.
 */
std::vector<Facet> generate_manifold_triangulation(
    int dimension,
    const std::function<double(const Eigen::VectorXd&)>& f,
    const std::vector<Eigen::VectorXd>& seeds,
    double scale
);