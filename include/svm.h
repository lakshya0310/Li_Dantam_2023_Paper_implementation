#pragma once
#include <thundersvm/dataset.h>
#include <thundersvm/svmparam.h>
#include <thundersvm/model/svc.h>
#include <thundersvm/util/metric.h>
#include <Eigen/Dense>

class Manifold {
public:
    Manifold(int dim) : dim_(dim) {}
    
    void train_samples(const std::vector<Eigen::VectorXd>& samples, std::vector<float>& labels);
    double operator()(const Eigen::VectorXd& q) const {
        std::vector<float> query_data(q.data(), q.data() + q.size());
        DataSet query_set;
        std::vector<float> dummy_label = {0.0f};
        query_set.load_from_dense(1, dim_, query_data.data(), dummy_label.data());
        
        SyncArray<float_type> dec_values;
        dec_values.resize(1);
        model_.predict_dec_values(query_set.instances(), dec_values, 1);
        
        const float_type* val_ptr = dec_values.host_data();
        if (dec_values.size() < 1) return 0.0;
        return (double)val_ptr[0];
    }

private:
    int dim_;
    SVC model_;
};