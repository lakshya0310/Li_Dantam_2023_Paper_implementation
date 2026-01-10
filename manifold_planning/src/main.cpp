#include "prm.hpp"
#include "sdcl.hpp"
#include "coxeter_triangulation.hpp"
#include "facet_checker.hpp"
#include "elastic_update.hpp"
#include "visualization.hpp"
#include <thread>
#include <iostream>
#include <filesystem>
#include <pybind11/embed.h>
#include <chrono>
#include <iomanip>
#include <filesystem>
namespace py = pybind11;
std::filesystem::create_directories("data");

static double seconds_since(const std::chrono::steady_clock::time_point& t0) {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now() - t0).count();
}


int main() {
    py::scoped_interpreter guard{};

    py::module sys = py::module::import("sys");
    sys.attr("path").attr("append")("simulations");
    RobotInterface* robot = new RobotInterface();

    PRM prm;
    SDCL sdcl(robot);

    std::thread sdcl_thread([&]() {
        while (true) {
            auto t0 = std::chrono::steady_clock::now();
            sdcl.update(prm);
            double t = seconds_since(t0);

        std::cout << "[TIMER] SDCL update (incl. SVM train) : "
                  << std::fixed << std::setprecision(6)
                  << t << " s\n";
        }
    });

    std::thread proof_thread([&]() {
        CoxeterTriangulation ct(robot->dof(), 0.1);
        ct.setManifoldFunction(
            [&](const std::vector<double>& q) {
                return robot->sdf(q);
            });

        while (true) {
        ct.setSeeds(sdcl.getManifoldSeeds());

        // ---- TRIANGULATION TIMER ----
        auto t0 = std::chrono::steady_clock::now();
        ct.traceManifold();
        double trace_t = seconds_since(t0);

        t0 = std::chrono::steady_clock::now();
        ct.constructTriangulation();
        double construct_t = seconds_since(t0);

        std::cout << "[TIMER] Triangulation trace      : "
                  << std::fixed << std::setprecision(6)
                  << trace_t << " s\n";
        std::cout << "[TIMER] Triangulation construct  : "
                  << std::fixed << std::setprecision(6)
                  << construct_t << " s\n";
        std::cout << "[TIMER] Triangulation total      : "
                  << std::fixed << std::setprecision(6)
                  << (trace_t + construct_t) << " s\n";

        auto check0 = std::chrono::steady_clock::now();

        ElasticUpdater updater(nullptr);
        FacetChecker checker(robot, &updater);

        bool ok = true;
        for (const auto& f : ct.facets()) {
            if (!checker.checkFacet(f)) {
                ok = false;
                break;
            }
        }

        double check_t = seconds_since(check0);
        std::cout << "[TIMER] Facet check total        : "
                  << std::fixed << std::setprecision(6)
                  << check_t << " s\n";

        if (ok) {
            std::cout << "[RESULT] Infeasibility proof found\n";
            break;
        }
    }
    });

    sdcl_thread.join();
    proof_thread.join();
    return 0;
}
