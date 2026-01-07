#pragma once
#include <vector>

/**
 * RBF SVM decision boundary F(q) = 0
 * Implemented with ThunderSVM
 */
class SVMManifold {
public:
    SVMManifold() = default;

    void train(const std::vector<std::vector<double>>& X,
               const std::vector<int>& y);

    /// Signed value of manifold function
    double evaluate(const std::vector<double>& q) const;

    bool trained() const { return trained_; }

private:
    bool trained_ = false;
};
