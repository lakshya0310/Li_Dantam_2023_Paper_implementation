#pragma once
#include "ImplicitModel.hpp"
#include <svm.h>   // libsvm

class SVMModel : public ImplicitModel {
    struct svm_model* model;
public:
    SVMModel(const char* filename);
    double operator()(const std::vector<double>& q) override;
};
