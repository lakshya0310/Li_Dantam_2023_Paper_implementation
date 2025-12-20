#pragma once
#include "SDFOracle.hpp"
#include <cstdio>

class PyBulletSDF : public SDFOracle {
    FILE* pipe;
public:
    PyBulletSDF(const char* sim) {
        pipe = popen(sim, "w");
    }
    ~PyBulletSDF() { pclose(pipe); }

    double operator()(const std::vector<double>& q) override {
        fprintf(pipe, "%f %f\n", q[0], q[1]);
        fflush(pipe);
        double d;
        fscanf(pipe, "%lf", &d);
        return d;
    }
};
