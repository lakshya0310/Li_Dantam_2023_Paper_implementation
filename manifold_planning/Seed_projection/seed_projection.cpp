#include <iostream>
#include <vector>
#include <functional>
#include <cmath>
#include <nlopt.hpp>
#include <Eigen/Dense>

struct OptData {
    std::function<double(const Eigen::VectorXd&)> F;
};


double objective_wrapper(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    OptData* opt_data = reinterpret_cast<OptData*>(data);
    Eigen::VectorXd vec_x = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());

    double val = opt_data->F(vec_x);
    double obj = val * val; 

    
    if (!grad.empty()) {
        double eps = 1e-5;
        for (size_t i = 0; i < x.size(); ++i) {
            Eigen::VectorXd x_plus = vec_x;
            x_plus[i] += eps;
            double val_plus = opt_data->F(x_plus);

            Eigen::VectorXd x_minus = vec_x;
            x_minus[i] -= eps;
            double val_minus = opt_data->F(x_minus);

    
            grad[i] = (val_plus * val_plus - val_minus * val_minus) / (2.0 * eps);
        }
    }
    return obj;
}

Eigen::VectorXd project_to_manifold(
    const Eigen::VectorXd& seed,
    const std::function<double(const Eigen::VectorXd&)>& F,
    double tol = 1e-6) 
{
    int dim = seed.size();
    

    nlopt::opt opt(nlopt::LD_SLSQP, dim);
    
    OptData data{F};
    opt.set_min_objective(objective_wrapper, &data);
    opt.set_ftol_abs(tol);
    opt.set_maxeval(200); 

    
    std::vector<double> x(seed.data(), seed.data() + dim);
    double minf;

    try {
        nlopt::result result = opt.optimize(x, minf);
    } catch (std::exception& e) {
    }

    return Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
}

int main() {
    
    auto spherical_manifold = [](const Eigen::VectorXd& q) {
        return q.squaredNorm() - 4.0; 
    };

    
    Eigen::VectorXd initial_seed(3);
    initial_seed << 0.2, 0.5, 0.1; 

    std::cout << "--- Manifold Projection Test ---" << std::endl;
    std::cout << "Initial Seed:   [" << initial_seed.transpose() << "]" << std::endl;
    std::cout << "Initial F(q):   " << spherical_manifold(initial_seed) << "\n" << std::endl;

    // Project!
    Eigen::VectorXd projected_seed = project_to_manifold(initial_seed, spherical_manifold);

    std::cout << "Projected Seed: [" << projected_seed.transpose() << "]" << std::endl;
    std::cout << "Final F(q):     " << spherical_manifold(projected_seed) << " (Target: 0.0)" << std::endl;
    std::cout << "Final Norm:     " << projected_seed.norm() << " (Target: 2.0)" << std::endl;

    return 0;
}