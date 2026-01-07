#pragma once
#include <vector>
#include <unordered_map>
#include <mutex>

struct CTSegment {
    std::vector<double> v1;
    std::vector<double> v2;
    std::vector<double> intersection;
};

class CoxeterTriangulation {
public:
    CoxeterTriangulation(size_t dim, double lambda_T);

    void setManifoldFunction(
        std::function<double(const std::vector<double>&)> f);

    void setSeeds(const std::vector<std::vector<double>>& seeds);

    /// Algorithm 1
    void traceManifold();

    /// Algorithm 3
    void constructTriangulation();

    const std::vector<std::vector<std::vector<double>>>& facets() const;

private:
    size_t dim_;
    double lambda_T_;

    std::function<double(const std::vector<double>&)> F_;
    std::vector<std::vector<double>> seeds_;

    std::vector<CTSegment> segments_;
    std::vector<std::vector<std::vector<double>>> facets_;

    std::mutex mutex_;
};
