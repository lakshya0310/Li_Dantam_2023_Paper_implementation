#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // Handles std::vector <-> Python list/numpy conversions safely
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

    // SAFE BINDING 1: Accept standard std::vector<double> from Python
    void add_sample(const std::vector<double>& q, float label) {
        if (q.size() != dim_) {
            std::cerr << "Error: Sample dimension mismatch! Actual dimension " << dim_ << " Input dimension " << q.size() << std::endl;
            return;
        }
        samples_.push_back(q);
        labels_.push_back(label);
    }

    void train_manifold() {
        if (samples_.empty()) return;

        std::cout << " [CPP] Preparing data for " << samples_.size() << " samples..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();

        // Flatten the safely stored std::vector<double> arrays
        std::vector<float> data;
        data.reserve(samples_.size() * dim_);
        for (const auto& s : samples_) {
            for (int i = 0; i < dim_; ++i) data.push_back((float)s[i]);
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
        std::cout << " [Timer] SVM Training: " << elapsed.count() << " s" << std::endl;
    }

    // INTERNAL: Eigen-based manifold function for high-speed C++ computation
    double manifold_function(const std::vector<double>& q) {
        DataSet::node2d instances(1);
        for(int i = 0; i < dim_; ++i){
            instances[0].emplace_back(i + 1, q[i]); 
        }
        
        SyncArray<float_type> dec_values(1);
        
        #pragma omp critical(svm_predict_lock)
        {
            model_.predict_dec_values(instances, dec_values, 1);
        }
        
        if (dec_values.size() < 1) return 0.0;
        return (double)dec_values.host_data()[0];
    }

    // SAFE BINDING 2: Python wrapper for manifold function
    double manifold_function_py(const std::vector<double>& q) {
        // Eigen::VectorXd vec = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
        return manifold_function(q);
    }

    // SAFE BINDING 3: Accept and return std::vector<double> structures to avoid Eigen segfaults
    std::vector<std::vector<std::vector<double>>> construct_proof(const std::vector<std::vector<double>>& python_seeds, double scale) {
        auto start = std::chrono::high_resolution_clock::now();
        // 1. Convert safe std::vectors to fast Eigen vectors
        std::vector<Eigen::VectorXd> eigen_seeds;
        for (const auto& s : python_seeds) {
            eigen_seeds.push_back(Eigen::Map<const Eigen::VectorXd>(s.data(), s.size()));
        }

        auto f = [this](const Eigen::VectorXd& x) {
            std::vector<double> point(x.size());
            for (int i = 0; i < x.size(); i++) {
                point[i] = x[i];
            }
            return this->manifold_function(point);
        };
        // 2. Run the core algorithm using Eigen
        auto facets = generate_manifold_triangulation(dim_, f, eigen_seeds, scale);
        // 3. Convert results back to safe std::vectors to hand back to Python
        std::vector<std::vector<std::vector<double>>> result;
        for (const auto& facet : facets) {
            std::vector<std::vector<double>> py_facet;
            for (const auto& vec : facet) {
                std::vector<double> pt(vec.data(), vec.data() + vec.size());
                py_facet.push_back(pt);
            }
            result.push_back(py_facet);
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << " [Timer] Coxeter Construction (Scale=" << scale << "): " << elapsed.count() << " s" << std::endl;

        return result;
    }

private:
    int dim_;
    // Now safely storing points as standard vectors, preventing Eigen ABI crashes
    std::vector<std::vector<double>> samples_; 
    std::vector<float> labels_;
    SVC model_; 
};

PYBIND11_MODULE(infeasibility_proof, m) {
    py::class_<InfeasibilityPlanner>(m, "InfeasibilityPlanner")
        .def(py::init<int>(), py::arg("dim"))

        .def("add_sample",
             &InfeasibilityPlanner::add_sample,
             py::arg("q"),
             py::arg("label"))

        .def("train_manifold",
             &InfeasibilityPlanner::train_manifold)

        .def("manifold_function",
             &InfeasibilityPlanner::manifold_function,
             py::arg("q"))

        .def("construct_proof",
             &InfeasibilityPlanner::construct_proof,
             py::arg("seeds"),
             py::arg("scale"));
}