#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <chrono>

// ThunderSVM Includes
#include <thundersvm/dataset.h>
#include <thundersvm/svmparam.h>
#include <thundersvm/model/svc.h>
#include <thundersvm/util/metric.h>

#include "coxeter_algo.h"

namespace py = pybind11;

class InfeasibilityPlanner {
public:
    InfeasibilityPlanner(int dim) : dim_(dim) {}

    // Add samples with custom labels
    void add_sample(const Eigen::VectorXd& q, float label) {
        samples_.push_back(q);
        labels_.push_back(label);
    }

    void train_manifold() {
        if (samples_.empty()) return;

        std::cout << "   [CPP] Preparing data for " << samples_.size() << " samples..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();

        std::vector<float> data;
        data.reserve(samples_.size() * dim_);
        for (const auto& s : samples_) {
            for (int i = 0; i < s.size(); ++i) data.push_back((float)s[i]);
        }

        SvmParam param;
        param.svm_type = SvmParam::C_SVC;
        param.kernel_type = SvmParam::RBF;
        param.gamma = 0.5; 
        param.C = 10;      
        param.probability = 1; 
        
        DataSet dataset;
        dataset.load_from_dense(samples_.size(), dim_, data.data(), labels_.data());
        
        // Train
        model_.train(dataset, param);
        
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "   [Timer] SVM Training: " << elapsed.count() << " s" << std::endl;
    }

    double manifold_function(const Eigen::VectorXd& q) {
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

    // Construct proof at F(x) = 0
    std::vector<Facet> construct_proof(const std::vector<Eigen::VectorXd>& seeds, double scale) {
        auto start = std::chrono::high_resolution_clock::now();

        auto f = [this](const Eigen::VectorXd& x) {
            return this->manifold_function(x);
        };
        
        auto result = generate_manifold_triangulation(dim_, f, seeds, scale);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "   [Timer] Coxeter Construction (Scale=" << scale << "): " << elapsed.count() << " s" << std::endl;

        return result;
    }

private:
    int dim_;
    std::vector<Eigen::VectorXd> samples_;
    std::vector<float> labels_;
    SVC model_; 
};

PYBIND11_MODULE(infeasibility_proof, m) {
    py::class_<InfeasibilityPlanner>(m, "InfeasibilityPlanner")
        .def(py::init<int>())
        .def("add_sample", &InfeasibilityPlanner::add_sample)
        .def("train_manifold", &InfeasibilityPlanner::train_manifold)
        .def("manifold_function", &InfeasibilityPlanner::manifold_function)
        .def("construct_proof", &InfeasibilityPlanner::construct_proof);
}