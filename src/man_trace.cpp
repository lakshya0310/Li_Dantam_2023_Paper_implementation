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
        double fp = f(p);
        if (std::abs(fp) <= eps) return p;
        if (fp * f1 > 0.0) {p1 = p; f1 = fp;}
        else {p2 = p; f2 = fp;}

        if ((p1 - p2).norm() < 1e-7) return 0.5 * (p1 + p2);
        p = (p1 * f2 - p2 * f1) / (f2 - f1);
    }

    return p;
}

void manifold_tracing(
    int dim,
    const std::function<double(const Eigen::VectorXd&)>& f,
    const std::vector<Eigen::VectorXd>& seeds,
    libcuckoo::cuckoohash_map<PermRep, Eigen::VectorXd, PermRepHash>& visited, 
    const CoxTri ct
) {
    std::vector<PermRep> frontier;
    frontier.reserve(1024);

    const double eps = 1e-8;
    #pragma omp parallel
    {
        std::vector<decltype(frontier)::value_type> local_frontier;

        #pragma omp for nowait
        for (size_t i = 0; i < seeds.size(); ++i) {
            const auto& s = seeds[i];

            auto simplex = ct.locate_point(s);

            for (const auto& edge : simplex.face_range(1)) {

                Eigen::VectorXd p1(dim), p2(dim);
                int cnt = 0;

                for (auto v : edge.vertex_range()) {
                    if (cnt == 0) p1 = ct.cartesian_coordinates(v);
                    else p2 = ct.cartesian_coordinates(v);
                    cnt++;
                }

                double f1 = f(p1);
                double f2 = f(p2);

                if (!((f1 > eps && f2 < -eps) || (f1 < -eps && f2 > eps)))
                    continue;

                Eigen::VectorXd ip = intersection_point(p1, p2, f);

                if (visited.insert(edge, ip)) {
                    local_frontier.push_back(edge);
                }
            }
        }

        // Merge once per thread
        #pragma omp critical
        {
            frontier.insert(frontier.end(),
                            local_frontier.begin(),
                            local_frontier.end());
        }
    }

    int iter = 0;
    const int MAX_ITERS = 1e6;

    while (!frontier.empty() && iter++ < MAX_ITERS) {

        std::vector<PermRep> next_frontier;
        next_frontier.reserve(frontier.size() * 2);

        #pragma omp parallel
        {
            std::vector<PermRep> local_next;
            local_next.reserve(512);

            Eigen::VectorXd p1(dim), p2(dim);

            #pragma omp for schedule(dynamic)
            for (int i = 0; i < (int)frontier.size(); i++) {

                const auto& edge = frontier[i];

                for (const auto& face : edge.coface_range(2)) {

                    for (const auto& new_edge : face.face_range(1)) {

                        if (visited.contains(new_edge)) continue;

                        int cnt = 0;
                        for (auto v : new_edge.vertex_range()) {
                            if (cnt == 0) p1 = ct.cartesian_coordinates(v);
                            else p2 = ct.cartesian_coordinates(v);
                            cnt++;
                        }

                        double f1 = f(p1);
                        double f2 = f(p2);

                        if (!((f1 > eps && f2 < -eps) || (f1 < -eps && f2 > eps)))
                            continue;

                        Eigen::VectorXd ip = intersection_point(p1, p2, f);

                        if (visited.insert(new_edge, ip)) {
                            local_next.push_back(new_edge);
                        }
                    }
                }
            }

            // Merge thread-local results
            #pragma omp critical
            next_frontier.insert(next_frontier.end(),
                                 local_next.begin(),
                                 local_next.end());
        }

        std::cout << "Frontier Size: " << frontier.size() << std::endl;

        frontier.swap(next_frontier);
    }
}

void manifold_meshing(
    int dim,
    CoxTri& ct,
    libcuckoo::cuckoohash_map<PermRep, Eigen::VectorXd, PermRepHash>& visited,
    libcuckoo::cuckoohash_map<PermRep, std::vector<Eigen::VectorXd>, PermRepHash>& mesh
) {
    // Need to treat the hash map sequentially for simplicial complex extraction
    auto locked_visited = visited.lock_table();

    for (const auto& kv : locked_visited) {
        const auto& edge = kv.first;
        const auto& ip = kv.second;

        for (const auto& tet : edge.coface_range(dim)) {
            mesh.upsert(
                tet,
                [&](std::vector<Eigen::VectorXd>& vec) {
                    for (const auto& p : vec) {
                        // To avoid repeated points as they create overlapping elements
                        if ((p - ip).norm() < 1e-4)
                            return;
                    }
                    vec.push_back(ip);
                },
                std::vector<Eigen::VectorXd>{ip}
            );
           }
    }
}