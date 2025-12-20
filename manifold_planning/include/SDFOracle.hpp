#pragma once
#include <vector>

class SDFOracle {
public:
    virtual double operator()(const std::vector<double>& q) = 0;
};
