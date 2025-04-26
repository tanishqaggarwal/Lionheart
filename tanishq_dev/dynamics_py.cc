#include "tanishq_dev/dynamics.h"
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>

namespace nb = nanobind;

NB_MODULE(dynamics_py, m) {
  nb::class_<Lionheart::Vector3<double>>(m, "Vector")
      .def(nb::init<double, double, double>())
      .def("to_numpy", [](const Lionheart::Vector3<double> &v) {
        double *data = new double[3];
        for (int i = 0; i < 3; i++) {
          data[i] = v(i);
        }

        return nb::ndarray<nb::numpy, double, nb::shape<3>>(
            data, {3}, nb::capsule(data, [](void *p) noexcept {
              delete[] static_cast<double *>(p);
            }));
      });

  nb::class_<Lionheart::Matrix3<double>>(m, "Matrix")
      .def(nb::init<const Lionheart::Vector3<double> &,
                    const Lionheart::Vector3<double> &,
                    const Lionheart::Vector3<double> &>())
      .def(nb::init<std::array<std::array<double, 3>, 3>>())
      .def("to_numpy", [](const Lionheart::Matrix3<double> &m) {
        double *data = new double[9];
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            data[i * 3 + j] = m(i, j);
          }
        }

        return nb::ndarray<nb::numpy, double, nb::shape<3, 3>>(
            data, {3, 3}, nb::capsule(data, [](void *p) noexcept {
              delete[] static_cast<double *>(p);
            }));
      });

  nb::class_<Lionheart::Rover::config_t>(m, "RoverConfig")
      .def(nb::init<double, double, const Lionheart::Matrix3<double> &,
                    const Lionheart::Vector3<double> &,
                    std::array<Lionheart::Vector3<double>,
                               Lionheart::Rover::config_t::N_THRUSTERS>,
                    std::array<Lionheart::Vector3<double>,
                               Lionheart::Rover::config_t::N_THRUSTERS>>(),
           nb::arg("mass"), nb::arg("volume"), nb::arg("moi"), nb::arg("cb"),
           nb::arg("thrust_positions"), nb::arg("thrust_vectors"));

  nb::class_<Lionheart::Rover::state_t>(m, "RoverState")
      .def(nb::init<const Lionheart::Vector3<double> &,
                    const Lionheart::Vector3<double> &,
                    const Lionheart::Vector3<double> &,
                    const Lionheart::Matrix3<double> &>(),
           nb::arg("position"), nb::arg("velocity"),
           nb::arg("angular_velocity"), nb::arg("attitude"));

  nb::class_<Lionheart::Rover>(m, "Rover")
      .def(nb::init<const Lionheart::Rover::state_t &, const Lionheart::Rover::config_t &>())
      .def("update", &Lionheart::Rover::update)
      .def("get_position", &Lionheart::Rover::get_position)
      .def("get_attitude", &Lionheart::Rover::get_attitude)
      .def("integrate_euler", &Lionheart::Rover::integrate_euler);
}