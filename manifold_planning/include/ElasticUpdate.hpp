#pragma once
#include <vector>
#include "SDFOracle.hpp"

std::vector<double> elasticUpdate(
    const std::vector<double>&,
    SDFOracle&,
    double alpha=0.05
);
