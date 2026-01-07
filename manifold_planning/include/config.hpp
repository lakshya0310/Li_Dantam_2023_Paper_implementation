#pragma once

#include <cstddef>

namespace config {

// ===== General =====
constexpr double EPSILON_B = 0.02;     // ε_b for facet checking
constexpr double TAU = 0.05;            // τ for manifold intersection
constexpr double LAMBDA_T_INIT = 0.1;   // Initial Coxeter λ_T
constexpr double LAMBDA_SHRINK = 0.9;   // Shrink factor

// ===== SVM =====
constexpr double SVM_C = 10.0;
constexpr double SVM_GAMMA = 0.5;

// ===== Optimization =====
constexpr int MAX_PROJECTION_ITERS = 50;

// ===== Parallelism =====
constexpr int MAX_THREADS = 32;

}
