#include "man_trace.h"

/*

CPU Parallelized manifold tracing and triangulation algorithm
Using the GUDHI library

Aayush Rath, Lakshya Jindal

*/

// Check if the edge intersects with the manifold
inline bool intersect_check(
    const Eigen::VectorXd& point_1,
    const Eigen::VectorXd& point_2,
    const std::function<double(const Eigen::VectorXd&)>& f
) {
    double f1 = f(point_1), f2 = f(point_2);
    return (f1 == 0.0) || (f2 == 0.0) || (f1 * f2 < 0.0);
}

// Find the point of intersection of the edge with the manifold using regula falsi 
Eigen::VectorXd intersection_point (
    const Eigen::VectorXd& point_1,
    const Eigen::VectorXd& point_2,
    const std::function<double(const Eigen::VectorXd&)>& f,
    const double eps = 1e-8,
    const int max_iter = 50
) {
    Eigen::VectorXd p1 = point_1, p2 = point_2;
    double f1 = f(p1), f2 = f(p2);

    // Edge cases when the vertex lies right on the manifold
    if (std::abs(f1) < eps) return p1;
    if (std::abs(f2) < eps) return p2;

    Eigen::VectorXd p = (p1 * f2 - p2 * f1) / (f2 - f1);

    // Iterative false position for computing the exact position
    for (int i = 0; i < max_iter; i++) {
        double fp;
        if (std::abs(fp) <= eps) return p;
        if (fp * f1 > 0.0) {p1 = p; f1 = fp;}
        else {p2 = p; f2 = fp;}

        if ((p1 - p2).norm() < 1e-7) return 0.5 * (p1 + p2);
        p = (p1 * f2 - p2 * f1) / (f2 - f1);
    }

    return p;
}

// BFS traversal for finding intersecting edges and intersection points
void manifold_tracing(
    int dim,
    const std::function<double(const Eigen::VectorXd&)>& f,
    const std::vector<Eigen::VectorXd>& seeds,
    libcuckoo::cuckoohash_map<PermRep, Eigen::VectorXd, PermRepHash>& visited, 
    const CoxTri ct
) {
    std::vector<PermRep> frontier;

    for (const auto& s : seeds) {
        Eigen::VectorXd sd_lattice = s;
        auto simplex = ct.locate_point(sd_lattice);
        for (auto& edge : simplex.face_range(1)) {
            if (!visited.contains(edge)) {
                frontier.push_back(edge);
                Eigen::VectorXd p1, p2;
                int cnt = 0;
                for (auto v : edge.vertex_range()) {
                    if (cnt == 0) p1 = ct.cartesian_coordinates(v);
                    else if (cnt == 1) p2 = ct.cartesian_coordinates(v);
                    cnt++;
                }

                Eigen::VectorXd ip = intersection_point(p1, p2, f);
            }
        }
    }

    // Frontier expansion loop
    while (!frontier.empty()) {
        // Storing a local version (each row) of the next frontier to avoid race conditions
        std::vector<std::vector<PermRep>> local_next(omp_get_max_threads());
        #pragma omp parallel for
        for (int i = 0; i < frontier.size(); i++) {
            int tid = omp_get_thread_num();
            auto edge = frontier[i];

            // Expanding to the 2d cofaces
            for (const auto& face : edge.coface_range(2)) {

                // 1d edges that make the current 2d simplex
                for (const auto& new_edge : face.face_range(1)) {
                    if (visited.contains(new_edge)) continue;

                    Eigen::VectorXd p1, p2;
                    int cnt = 0;
                    for (auto v : edge.vertex_range()) {
                        if (cnt == 0) p1 = ct.cartesian_coordinates(v);
                        else if (cnt == 1) p2 = ct.cartesian_coordinates(v);
                        cnt++;
                    }

                    if (!intersect_check(p1, p2, f)) continue;
                    visited.insert(new_edge, intersection_point(p1, p2, f));
                    local_next[tid].push_back(new_edge);
                }
            }
        }
        frontier.clear();
        for (auto& v : local_next)
        frontier.insert(frontier.end(), v.begin(), v.end());
    }
}

void manifold_meshing(
    int dim,
    CoxTri& ct,
    libcuckoo::cuckoohash_map<PermRep, Eigen::VectorXd, PermRepHash>& visited,
    libcuckoo::cuckoohash_map<PermRep, std::vector<Eigen::VectorXd>, PermRepHash>& mesh
) {
    auto locked_visited = visited.lock_table();

    for (const auto& kv : locked_visited) {
        const auto& edge = kv.first;
        const auto& ip = kv.second;

        for (const auto& tet : edge.coface_range(dim)) {
            mesh.upsert(
                tet,
                [&](std::vector<Eigen::VectorXd>& vec) {
                    for (const auto& p : vec) {
                        if ((p - ip).norm() < 1e-6)
                            return;
                    }
                    vec.push_back(ip);
                },
                std::vector<Eigen::VectorXd>{ip}
            );
        }
    }
}