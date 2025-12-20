#pragma once
#include <vector>
#include <array>

struct Simplex { std::array<int,3> v; };

class CoxeterTriangulation {
public:
    std::vector<std::vector<double>> vertices;
    std::vector<Simplex> simplices;

    CoxeterTriangulation(double qmin, double qmax, double lambda);
};
