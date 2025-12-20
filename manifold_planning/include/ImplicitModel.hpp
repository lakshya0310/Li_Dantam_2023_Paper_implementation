#pragma once
#include <vector>

class ImplicitModel {
public:
    virtual double operator()(const std::vector<double>& q) = 0;
};
