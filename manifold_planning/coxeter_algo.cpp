#include "coxeter_algo.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <omp.h> // Include OpenMP for parallelization
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
    double f1, f2;
    // Protect ThunderSVM inference from concurrent thread clashes
    #pragma omp critical(svm_predict)
    {
        f1 = f(p1);
        f2 = f(p2);
    }
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
    
    double f1, f2;
    #pragma omp critical(svm_predict)
    {
        f1 = f(p1);
        f2 = f(p2);
    }

    if (f1 * f2 > 0.0) return 0.5 * (p1 + p2);
    if (std::abs(f1) <= eps) return p1;
    if (std::abs(f2) <= eps) return p2;

    Eigen::VectorXd p = (p1 * f2 - p2 * f1) / (f2 - f1);

    for (int i = 0; i < max_iter; ++i) {
        double fp;
        #pragma omp critical(svm_predict)
        {
            fp = f(p);
        }
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
    
    // Use Level-Synchronous BFS for Parallelization
    std::vector<PR> current_level;
    std::unordered_set<PR, PRHash, PREq> visited_edges;
    std::unordered_map<PR, Eigen::VectorXd, PRHash, PREq> Ps; 

    auto f_scaled = [&](const Eigen::VectorXd& y) {
        return f(y * scale);
    };
    // 1. Initialization
    for (const auto& s : seeds) {
        Eigen::VectorXd s_lattice = s / scale; 
        auto simplex = ct.locate_point(s_lattice);
        for (auto edge : simplex.face_range(1)) {
            if (visited_edges.insert(edge).second) {
                current_level.push_back(edge);
            }
        }
    }

    // 2. Parallel Manifold Tracing (Algorithm 1)
    while (!current_level.empty()) {
        std::vector<PR> next_level;

        // Parallelize exploring neighbors of the current BFS level
        #pragma omp parallel
        {
            std::vector<PR> local_next;
            
            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < current_level.size(); ++i) {
                PR ls = current_level[i];

                for (auto c : ls.coface_range(2)) {
                    for (auto l : c.face_range(1)) {
                        
                        bool is_visited;
                        #pragma omp critical(visit_check)
                        {
                            is_visited = (visited_edges.find(l) != visited_edges.end());
                        }
                        if (is_visited) continue;

                        Eigen::VectorXd p1_lat, p2_lat;
                        int cnt = 0;
                        for (auto v : l.vertex_range()) {
                            if (cnt == 0) p1_lat = ct.cartesian_coordinates(v);
                            else if (cnt == 1) p2_lat = ct.cartesian_coordinates(v);
                            cnt++;
                        }

                        if (intersect_check(p1_lat, p2_lat, f_scaled)) {
                            bool is_new = false;
                            
                            #pragma omp critical(visit_check)
                            {
                                if (visited_edges.insert(l).second) {
                                    is_new = true;
                                }
                            }

                            if (is_new) {
                                Eigen::VectorXd ip_lat = intersect(p1_lat, p2_lat, f_scaled);
                                
                                #pragma omp critical(ps_insert)
                                {
                                    Ps.emplace(l, ip_lat * scale);
                                }
                                local_next.push_back(l);
                            }
                        }
                    }
                }
            }
            
            // Merge local queue into global next level
            #pragma omp critical(merge_queues)
            {
                next_level.insert(next_level.end(), local_next.begin(), local_next.end());
            }
        }
        current_level = next_level;
    }

    // 3. Parallel Construct Triangulation (Algorithm 3)
    std::unordered_map<PR, std::vector<Eigen::VectorXd>, PRHash, PREq> M;
    std::vector<std::pair<PR, Eigen::VectorXd>> ps_vec(Ps.begin(), Ps.end());

    #pragma omp parallel
    {
        std::unordered_map<PR, std::vector<Eigen::VectorXd>, PRHash, PREq> local_M;
        
        #pragma omp for schedule(static)
        for (size_t i = 0; i < ps_vec.size(); ++i) {
            const PR& edge = ps_vec[i].first;
            const Eigen::VectorXd& ip_world = ps_vec[i].second;
            for (auto c : edge.coface_range(dimension)) {
                local_M[c].push_back(ip_world);
            }
        }

        #pragma omp critical(m_merge)
        {
            for (const auto& kv : local_M) {
                for (const auto& p : kv.second) {
                    M[kv.first].push_back(p);
                }
            }
        }
    }

    // Convert to Facets
    std::vector<Facet> facets;
    for (const auto& kv : M) {
        if (kv.second.size() >= (size_t)dimension) {
            facets.push_back(kv.second);
        }
    }
    
    return facets;
}