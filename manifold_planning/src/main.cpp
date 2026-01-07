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
namespace py = pybind11;
std::filesystem::create_directories("data");

int main() {
    py::scoped_interpreter guard{};

    py::module sys = py::module::import("sys");
    sys.attr("path").attr("append")("simulations");
    RobotInterface* robot = new RobotInterface();

    PRM prm;
    SDCL sdcl(robot);

    std::thread sdcl_thread([&]() {
        while (true) {
            sdcl.update(prm);
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
            ct.traceManifold();
            ct.constructTriangulation();

            ElasticUpdater updater(nullptr);
            FacetChecker checker(robot, &updater);

            for (const auto& f : ct.facets()) {
                if (!checker.checkFacet(f))
                    break;
            }

            std::cout << "Infeasibility proof found\n";
            break;
        }
    });

    sdcl_thread.join();
    proof_thread.join();
    return 0;
}
