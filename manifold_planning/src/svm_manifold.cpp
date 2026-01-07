#include "svm_manifold.hpp"
#include <thundersvm/svm.h>

using namespace thundersvm;

static SvmModel model;

void SVMManifold::train(const std::vector<std::vector<double>>& X,
                        const std::vector<int>& y) {
    Dataset dataset;
    dataset.load_from_dense(X, y);

    SvmParam param;
    param.kernel_type = RBF;
    param.C = 10.0;
    param.gamma = 0.5;
    param.max_iter = 1000;

    model.train(dataset, param);
    trained_ = true;
}

double SVMManifold::evaluate(const std::vector<double>& q) const {
    if (!trained_) return 0.0;
    return model.predict(q);
}
