#include "robot_interface.hpp"
#include <pybind11/embed.h>

namespace py = pybind11;

class PyBulletRobotImpl {
public:
    py::object sdf_func;
    py::object collision_func;
    size_t dof;

    PyBulletRobotImpl(const std::string& module) {
        py::module m = py::module::import(module.c_str());
        sdf_func = m.attr("sdf");
        collision_func = m.attr("collision");
        dof = m.attr("DOF").cast<size_t>();
    }
};

static py::scoped_interpreter guard{};
static PyBulletRobotImpl* impl = nullptr;

double RobotInterface::sdf(const std::vector<double>& q) {
    return impl->sdf_func(q).cast<double>();
}

bool RobotInterface::inCollision(const std::vector<double>& q) {
    return impl->collision_func(q).cast<bool>();
}

size_t RobotInterface::dof() const {
    return impl->dof;
}
