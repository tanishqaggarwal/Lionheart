#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include "tanishq_dev/dynamics.h"

namespace nb = nanobind;

NB_MODULE(dynamics_py, m) {
    nb::class_<Lionheart::Vector3<double>>(m, "Vector")
        .def(nb::init<>());
    nb::class_<Lionheart::Matrix3<double>>(m, "Matrix")
        .def(nb::init<>());
    nb::class_<Lionheart::Rover::config_t>(m, "RoverConfig")
        .def(nb::init<>());
    nb::class_<Lionheart::Rover>(m, "Rover")
        .def(nb::init<const Lionheart::Rover::config_t&>())
        .def("update", &Lionheart::Rover::update);
}