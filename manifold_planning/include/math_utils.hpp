#pragma once
#include <vector>
#include <cmath>
#include <utility>

inline double l2_norm(const std::vector<double>& v) {
    double s = 0.0;
    for (double x : v) s += x * x;
    return std::sqrt(s);
}

inline std::vector<double> midpoint(
    const std::vector<double>& a,
    const std::vector<double>& b) {

    std::vector<double> m(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        m[i] = 0.5 * (a[i] + b[i]);
    return m;
}

inline size_t longest_edge(
    const std::vector<std::vector<double>>& simplex) {

    double max_d = -1.0;
    size_t idx = 0;
    for (size_t i = 0; i < simplex.size(); ++i) {
        for (size_t j = i + 1; j < simplex.size(); ++j) {
            double d = 0.0;
            for (size_t k = 0; k < simplex[i].size(); ++k) {
                double t = simplex[i][k] - simplex[j][k];
                d += t * t;
            }
            if (d > max_d) {
                max_d = d;
                idx = i * simplex.size() + j;
            }
        }
    }
    return idx;
}
