#include "coxeter_triangulation.hpp"
#include <gudhi/Coxeter_triangulation.h>
#include <cmath>

using CT = Gudhi::coxeter_triangulation::Coxeter_triangulation<double>;

CoxeterTriangulation::CoxeterTriangulation(size_t dim, double lambda_T)
    : dim_(dim), lambda_T_(lambda_T) {}

void CoxeterTriangulation::setManifoldFunction(
    std::function<double(const std::vector<double>&)> f) {
    F_ = f;
}

void CoxeterTriangulation::setSeeds(
    const std::vector<std::vector<double>>& seeds) {
    seeds_ = seeds;
}

static std::vector<double> intersect(
    const std::vector<double>& a,
    const std::vector<double>& b,
    const std::function<double(const std::vector<double>&)>& F) {

    std::vector<double> v1 = a;
    std::vector<double> v2 = b;

    for (int i = 0; i < 50; ++i) {
        std::vector<double> mid(v1.size());
        for (size_t k = 0; k < v1.size(); ++k)
            mid[k] = 0.5 * (v1[k] + v2[k]);

        if (F(v1) * F(mid) < 0)
            v2 = mid;
        else
            v1 = mid;
    }
    return v1;
}

void CoxeterTriangulation::traceManifold() {
    CT ct(dim_, lambda_T_);

    #pragma omp parallel for
    for (size_t i = 0; i < seeds_.size(); ++i) {
        auto simplex = ct.locate(seeds_[i]);
        for (auto edge : simplex.cofaces(1)) {
            auto v = edge.vertices();
            if (F_(v[0]) * F_(v[1]) < 0) {
                CTSegment seg;
                seg.v1 = v[0];
                seg.v2 = v[1];
                seg.intersection = intersect(v[0], v[1], F_);

                std::lock_guard<std::mutex> lock(mutex_);
                segments_.push_back(seg);
            }
        }
    }
}

void CoxeterTriangulation::constructTriangulation() {
    std::unordered_map<size_t, std::vector<std::vector<double>>> cell_map;

    for (const auto& s : segments_) {
        size_t key = std::hash<double>{}(s.intersection[0]);
        cell_map[key].push_back(s.intersection);
    }

    for (auto& kv : cell_map) {
        if (kv.second.size() >= dim_) {
            facets_.push_back(kv.second);
        }
    }
}

const std::vector<std::vector<std::vector<double>>>&
CoxeterTriangulation::facets() const {
    return facets_;
}
