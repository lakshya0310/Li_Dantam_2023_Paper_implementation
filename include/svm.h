#pragma once
#include <thundersvm/dataset.h>
#include <thundersvm/svmparam.h>
#include <thundersvm/model/svc.h>
#include <thundersvm/util/metric.h>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Manifold — RBF-SVM wrapper with zero-overhead native evaluation
//
// After training via ThunderSVM, we extract the support vectors, dual
// coefficients (α_i * y_i), and bias ρ, then evaluate the decision function
// entirely in Eigen:
//
//   f(q) = Σ_i coef_i · exp(-γ ‖q - sv_i‖²) - ρ
//
// This replaces all operator() and batch_predict calls with pure arithmetic —
// no DataSet construction, no predict_dec_values, no ThunderSVM overhead at all.
// Each evaluation is O(n_sv * dim), typically ~1–5 µs vs ~500 µs for
// ThunderSVM's per-call overhead observed in the log.
// ─────────────────────────────────────────────────────────────────────────────

class SVCExpose : public SVC {
public:
    int get_n_sv() const { return this->n_total_sv; }

    const SyncArray<float_type>& get_coef() const { return this->coef; }
    const SyncArray<float_type>& get_rho()  const { return this->rho; }

    const DataSet::node2d& get_sv() const { return this->sv; }

    float get_gamma() const { return this->param.gamma; }
};

class Manifold {
public:
    explicit Manifold(int dim) : dim_(dim), gamma_(0.5), rho_(0.0) {}

    void train_samples(const std::vector<Eigen::VectorXd>& samples,
                       std::vector<float>& labels);

    // Single-point native evaluation — O(n_sv * dim), no ThunderSVM overhead.
    double operator()(const Eigen::VectorXd& q) const {
        double result = -rho_;
        for (int i = 0; i < (int)sv_.rows(); ++i) {
            double diff_sq = (sv_.row(i).transpose() - q).squaredNorm();
            result += coef_[i] * std::exp(-gamma_ * diff_sq);
        }
        return result;
    }

    // Batch evaluation — evaluates n points stored row-major in `rows`.
    // Still zero ThunderSVM overhead; runs the kernel loop n times.
    // Used by the SLSQP gradient step and the BFS endpoint batch.
    std::vector<double> batch_predict(const std::vector<float>& rows, int n) const {
        std::vector<double> out(n);
        int d = dim_;
        for (int k = 0; k < n; ++k) {
            Eigen::VectorXd q(d);
            for (int j = 0; j < d; ++j) q[j] = (double)rows[k * d + j];
            out[k] = (*this)(q);
        }
        return out;
    }

private:
    int              dim_;
    double           gamma_;
    double           rho_;
    Eigen::MatrixXd  sv_;    // (n_sv × dim)  support vectors, row-major
    std::vector<double> coef_;  // (n_sv)  α_i * y_i
    SVCExpose model_;
};