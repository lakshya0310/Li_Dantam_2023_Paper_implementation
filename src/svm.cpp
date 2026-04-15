#include "svm.h"


void Manifold::train_samples(const std::vector<Eigen::VectorXd>& samples, std::vector<float>& labels) {
        if (samples.empty()) return;

        std::cout << "   [CPP] Preparing data for " << samples.size() << " samples..." << std::endl;
        std::vector<float> data;
        data.reserve(samples.size() * dim_);
        for (const auto& s : samples) {
            for (int i = 0; i < s.size(); ++i) data.push_back((float)s[i]);
        }

        SvmParam param;
        param.svm_type = SvmParam::C_SVC;
        param.kernel_type = SvmParam::RBF;
        param.gamma = 0.5; 
        param.C = 10;      
        param.probability = 1; 
        
        DataSet dataset;
        dataset.load_from_dense(samples.size(), dim_, data.data(), labels.data());
        
        this->model_.train(dataset, param);
}