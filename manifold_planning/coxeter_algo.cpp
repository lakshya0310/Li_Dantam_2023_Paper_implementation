#include "coxeter_algo.h"
#include <iostream>
#include <sstream>
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <gudhi/Coxeter_triangulation.h>

using PR = Gudhi::coxeter_triangulation::Permutahedral_representation<
    std::vector<int>,
    std::vector<std::vector<std::size_t>>
>;
using CT = Gudhi::coxeter_triangulation::Coxeter_triangulation<>;

// ---- Hashing Utilities ----
static inline std::string pr_to_string(const PR& pr) {
    std::ostringstream oss;
    oss << pr;
    return oss.str();
}

struct PRHash {
    std::size_t operator()(const PR& pr) const noexcept {
        return std::hash<std::string>{}(pr_to_string(pr));
    }
};

struct PREq {
    bool operator()(const PR& a, const PR& b) const noexcept {
        return a == b;
    }
};

// ---- Algorithms 1 & 2 Helpers ----

bool intersect_check(
    const Eigen::VectorXd& p1,
    const Eigen::VectorXd& p2,
    const std::function<double(const Eigen::VectorXd&)>& f
) {
    double f1 = f(p1);
    double f2 = f(p2);
    return (f1 == 0.0) || (f2 == 0.0) || (f1 * f2 < 0.0);
}

Eigen::VectorXd intersect(
    const Eigen::VectorXd& v1,
    const Eigen::VectorXd& v2,
    const std::function<double(const Eigen::VectorXd&)>& f,
    double eps = 1e-5,
    int max_iter = 50
) {
    Eigen::VectorXd p1 = v1;
    Eigen::VectorXd p2 = v2;
    double f1 = f(p1);
    double f2 = f(p2);

    if (f1 * f2 > 0.0) return 0.5 * (p1 + p2);
    if (std::abs(f1) <= eps) return p1;
    if (std::abs(f2) <= eps) return p2;

    Eigen::VectorXd p = (p1 * f2 - p2 * f1) / (f2 - f1);

    for (int i = 0; i < max_iter; ++i) {
        double fp = f(p);
        if (std::abs(fp) <= eps) return p;
        
        if (fp * f1 > 0.0) {
            p1 = p; f1 = fp;
        } else {
            p2 = p; f2 = fp;
        }
        
        if ((p1 - p2).norm() < 1e-7) return 0.5 * (p1 + p2);
        p = (p1 * f2 - p2 * f1) / (f2 - f1);
    }
    return p;
}

// ---- Main Function Implementation ----

std::vector<Facet> generate_manifold_triangulation(
    int dimension,
    const std::function<double(const Eigen::VectorXd&)>& f,
    const std::vector<Eigen::VectorXd>& seeds,
    double scale
) {
    CT ct(dimension); 
    std::queue<PR> q;
    std::unordered_set<PR, PRHash, PREq> visited_edges;
    std::unordered_map<PR, Eigen::VectorXd, PRHash, PREq> Ps; 

    // WRAPPER: Map Lattice space (y) to World space (x)
    // x = scale * y
    auto f_scaled = [&](const Eigen::VectorXd& y) {
        return f(y * scale);
    };

    // 1. Initialization with ALL seeds (converted to lattice space)
    for (const auto& s : seeds) {
        Eigen::VectorXd s_lattice = s / scale; 
        auto simplex = ct.locate_point(s_lattice);
        for (auto edge : simplex.face_range(1)) {
            if (visited_edges.insert(edge).second) {
                q.push(edge);
            }
        }
    }

    // 2. Manifold Tracing (Algorithm 1) in Lattice Space
    while (!q.empty()) {
        PR ls = q.front();
        q.pop();

        for (auto c : ls.coface_range(2)) {
            for (auto l : c.face_range(1)) {
                Eigen::VectorXd p1_lat, p2_lat;
                int cnt = 0;
                for (auto v : l.vertex_range()) {
                    if (cnt == 0) p1_lat = ct.cartesian_coordinates(v);
                    else if (cnt == 1) p2_lat = ct.cartesian_coordinates(v);
                    cnt++;
                }

                if (intersect_check(p1_lat, p2_lat, f_scaled)) {
                    if (Ps.find(l) == Ps.end()) {
                        // Calculate intersection in LATTICE space
                        Eigen::VectorXd ip_lat = intersect(p1_lat, p2_lat, f_scaled);
                        // Store intersection in WORLD space
                        Ps.emplace(l, ip_lat * scale);
                    }
                    if (visited_edges.insert(l).second) {
                        q.push(l);
                    }
                }
            }
        }
    }

    // 3. Construct Triangulation (Algorithm 3)
    std::unordered_map<PR, std::vector<Eigen::VectorXd>, PRHash, PREq> M;
    for (const auto& kv : Ps) {
        const PR& edge = kv.first;
        const Eigen::VectorXd& ip_world = kv.second;
        for (auto c : edge.coface_range(dimension)) {
            M[c].push_back(ip_world);
        }
    }

    std::vector<Facet> facets;
    for (const auto& kv : M) {
        // Only keep full facets (dimension vertices for codimension-1)
        if (kv.second.size() >= (size_t)dimension) {
            facets.push_back(kv.second);
        }
    }
    
    return facets;
}