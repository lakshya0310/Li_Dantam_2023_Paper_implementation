#include "SVMModel.hpp"

SVMModel::SVMModel(const char* f){
    model = svm_load_model(f);
}

double SVMModel::operator()(const std::vector<double>& q){
    svm_node nodes[3];
    nodes[0] = {1, q[0]};
    nodes[1] = {2, q[1]};
    nodes[2] = {-1, 0};
    return svm_predict(model, nodes);
}
