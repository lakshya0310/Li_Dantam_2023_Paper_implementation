#include "svm.h"
#include <stdexcept>
#include <iostream>

void Manifold::train_samples(const std::vector<Eigen::VectorXd>& samples,
                              std::vector<float>& labels)
{
    if (samples.empty()) return;

    std::cout << "   [CPP] Preparing data for " << samples.size()
              << " samples..." << std::endl;

    std::vector<float> data;
    data.reserve(samples.size() * dim_);
    for (const auto& s : samples)
        for (int i = 0; i < (int)s.size(); ++i)
            data.push_back((float)s[i]);

    SvmParam param;
    param.svm_type    = SvmParam::C_SVC;
    param.kernel_type = SvmParam::RBF;
    param.gamma       = (float)gamma_;
    param.C           = 1e5;
    param.probability = 0;

    DataSet dataset;
    dataset.load_from_dense(samples.size(), dim_, data.data(), labels.data());
    model_.train(dataset, param);

    // ── Extract kernel parameters directly from model_ ────────────────────────
    // SvmModel public members used here:
    //   model_.n_total_sv          : int    — total number of support vectors
    //   model_.sv                  : DataSet::node2d — sparse SVs (vector of
    //                                  vector<DataSet::node>), each node has
    //                                  .index (1-based) and .value (float)
    //   model_.coef                : SyncArray<float_type> — flat array of
    //                                  dual coefficients α_i*y_i, size
    //                                  (n_classes-1) * n_total_sv.  For binary
    //                                  classification n_classes=2 so it is
    //                                  exactly n_total_sv coefficients.
    //   model_.rho                 : SyncArray<float_type> — bias terms, one
    //                                  per binary classifier.  For binary SVC
    //                                  this is a single value.
    //   model_.param.gamma         : float — the γ used during training
    //                                  (confirms what we set above)

    int n_sv = model_.get_n_sv();

    // ── Dual coefficients α_i * y_i ──────────────────────────────────────────
    // coef is on the GPU/CPU SyncArray; host_data() syncs and returns a ptr.
    const float_type* coef_ptr = model_.get_coef().host_data();
    coef_.resize(n_sv);
    for (int i = 0; i < n_sv; ++i)
        coef_[i] = (double)coef_ptr[i];

    // ── Bias ρ ────────────────────────────────────────────────────────────────
    const float_type* rho_ptr = model_.get_rho().host_data();
    rho_ = (double)rho_ptr[0];


    // ── Support vectors (sparse → dense) ─────────────────────────────────────
    // model_.sv is a DataSet::node2d = vector<vector<DataSet::node>>
    // Each inner vector is one support vector; DataSet::node has:
    //   .index  (int,   1-based feature index)
    //   .value  (float, feature value)
    // We densify into an (n_sv × dim_) Eigen matrix.
    const DataSet::node2d& svs = model_.get_sv();
    sv_.setZero(n_sv, dim_);
    for (int i = 0; i < n_sv; ++i) {
        for (const auto& node : svs[i]) {
            int col = node.index - 1;   // 1-based → 0-based
            if (col >= 0 && col < dim_)
                sv_(i, col) = (double)node.value;
        }
    }

    std::cout << "   [Native] Extracted " << n_sv
              << " SVs, gamma=" << gamma_
              << ", rho=" << rho_ << std::endl;
}