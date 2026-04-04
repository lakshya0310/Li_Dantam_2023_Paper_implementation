#ifndef MAN_TRACE_H
#define MAN_TRACE_H

#include <vector>
#include <functional>
#include <Eigen/Dense>
#include <gudhi/Coxeter_triangulation.h>
#include <libcuckoo/cuckoohash_map.hh>
#include <omp.h>

// Name shortening for convienience
using PermRep = Gudhi::coxeter_triangulation::Permutahedral_representation<std::vector<int>, std::vector<std::vector<size_t>>>;
using CoxTri = Gudhi::coxeter_triangulation::Coxeter_triangulation<>;

inline void hash_combine(size_t& seed, size_t h) {
    seed ^= h + 0x9e3779b97f4a7c15 + (seed << 6) + (seed >> 2);
}

struct PermRepHash {
    size_t operator()(const PermRep& p) const {
        size_t seed = 0;
        for (int v : p.vertex())
            hash_combine(seed, std::hash<int>()(v));

        for (const auto& part : p.partition()) {
            for (size_t x : part)
                hash_combine(seed, std::hash<size_t>()(x));
            hash_combine(seed, 0x9e3779b9);
        }

        return seed;
    }
};

void manifold_tracing(
    int dim,
    const std::function<double(const Eigen::VectorXd&)>& f,
    const std::vector<Eigen::VectorXd>& seeds,
    libcuckoo::cuckoohash_map<PermRep, Eigen::VectorXd, PermRepHash>& visited, 
    const CoxTri ct
);
void manifold_meshing(
    int dim,
    CoxTri& ct,
    libcuckoo::cuckoohash_map<PermRep, Eigen::VectorXd, PermRepHash>& visited,
    libcuckoo::cuckoohash_map<PermRep, std::vector<Eigen::VectorXd>, PermRepHash>& mesh
);

#endif