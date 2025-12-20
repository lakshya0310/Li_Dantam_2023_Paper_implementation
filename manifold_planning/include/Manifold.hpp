#pragma once
#include <vector>
#include "CoxeterTriangulation.hpp"
#include "SDFOracle.hpp"

struct Edge { int a,b; };

class Manifold {
public:
    std::vector<Edge> edges;
    void extract(const CoxeterTriangulation&, SDFOracle&);
};
