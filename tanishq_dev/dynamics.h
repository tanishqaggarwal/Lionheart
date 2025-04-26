#pragma once

#include <array>
#include <cmath>

#include "tanishq_dev/linalg.h"

namespace Lionheart {

/// @brief Implements 6DOF dynamics for rover.
template <typename T> class RoverT {
private:
  // Internal state of system
  template <typename Quantity> struct integrated_state_t {
    Quantity value;
    Quantity derivative;
  };

public:
  struct state_t {
    /// Position and velocity of CG, inertial frame.
    integrated_state_t<Vector3<T>> position;
    integrated_state_t<Vector3<T>> velocity;

    /// Angular velocity of body w.r.t. CG, body frame.
    integrated_state_t<Vector3<T>> angular_velocity;

    /// DCM translating vectors from body frame to inertial frame.
    integrated_state_t<Matrix3<T>> attitude;

    state_t(const Vector3<T> &initial_position,
            const Vector3<T> &initial_velocity,
            const Vector3<T> &initial_angular_velicity,
            const Matrix3<T> &initial_attitude)
        : position({.value = initial_position, .derivative = Vector3<T>()}),
          velocity({.value = initial_velocity, .derivative = Vector3<T>()}),
          angular_velocity(
              {.value = initial_angular_velicity, .derivative = Vector3<T>()}),
          attitude({.value = initial_attitude, .derivative = Matrix3<T>()}) {}
  };

private:
  state_t state;

public:
  struct config_t {
    /// Density of seawater in the operating region
    static constexpr T water_density{1035.0}; // kg / m^3

    /// Inertial acceleration due to gravity
    static constexpr Vector3<T> accel_gravity{0, 0, 9.81}; // m/s^2

    /// Total mass of rover (accounting for internal water storage as well.)
    T mass{0.0};

    /// Total dry volume of rover.
    T volume{0.0};

    /// Moment of inertia (and its inverse) in body frame.
    Matrix3<T> moi;
    Matrix3<T> moi_inv;

    /// Location of center of buoyancy relative to CG in body frame.
    Vector3<T> cb;

    /// Positions and directions of thrust vectors, in body frame.
    static constexpr size_t N_THRUSTERS{5};
    std::array<Vector3<T>, N_THRUSTERS> thrust_positions;
    std::array<Vector3<T>, N_THRUSTERS> thrust_vectors;

    /// Constructor
    config_t(T mass_, T volume_, const Matrix3<T> &moi_, const Vector3<T> &cb_,
             const std::array<Vector3<T>, N_THRUSTERS> &thrust_positions_,
             const std::array<Vector3<T>, N_THRUSTERS> &thrust_vectors_)
        : mass(mass_), volume(volume_), moi(moi_), moi_inv(moi_.inverse()),
          cb(cb_), thrust_positions(thrust_positions_),
          thrust_vectors(thrust_vectors_) {}
  } config;

  RoverT(const state_t &initial_state, const config_t &config_)
      : state(initial_state), config(config_) {}

  // Compute the "effective" mass and MOI due to motion through water.
  // TODO
  // Assumptions:
  // - Water is inviscid and incompressible
  // - Make some assumptions about body geometry
  std::tuple<Matrix3<T>, T> added_mass() const {
    return std::make_tuple(Matrix3<T>{}, T{0.0});
  }

  void update(const std::array<T, config_t::N_THRUSTERS> &thrusts) {
    // Get "added mass" effects.
    const auto [moi_added, m_added] = added_mass();

    // Get net thrust force in body frame.
    Vector3<T> net_thrust_force;
    for (size_t i = 0; i < config_t::N_THRUSTERS; i++) {
      net_thrust_force += config.thrust_vectors[i] * thrusts[i];
    }

    // Get net thrust torque in body frame.
    Vector3<T> net_thrust_torque;
    for (size_t i = 0; i < config_t::N_THRUSTERS; i++) {
      net_thrust_torque += config.thrust_positions[i].cross(
          config.thrust_vectors[i] * thrusts[i]);
    }

    // Get net buoyancy
    Vector3<T> buoyancy_force = -config_t::water_density * config.volume *
                                config_t::accel_gravity; // Inertial frame
    Vector3<T> buoyancy_torque = config.cb.cross(
        state.attitude.value.inverse() * buoyancy_force); // Body frame

    // TODO compute drag force, current disturbances

    // Aggregate torques and forces
    Vector3<T> net_torque_body = buoyancy_torque + net_thrust_torque;
    Vector3<T> net_force_inertial =
        buoyancy_force + state.attitude.value * net_thrust_force;

    // Compute equations of motion
    state.position.derivative = state.velocity.value;
    state.velocity.derivative = net_force_inertial / (config.mass + m_added);
    state.angular_velocity.derivative =
        config.moi_inv *
        (net_torque_body - state.angular_velocity.value.cross(
                               config.moi * state.angular_velocity.value));

    const auto &w = state.angular_velocity.value;
    state.attitude.derivative =
        state.attitude.value *
        Matrix3<T>{{0, -w.z, w.y}, {w.z, 0, -w.x}, {-w.y, w.x, 0}};
  }

  template <typename IntegratorFn>
  void integrate(T dt, IntegratorFn integrator) {
    state.position.value =
        integrator(state.position.value, state.position.derivative, dt);
    state.velocity.value =
        integrator(state.velocity.value, state.velocity.derivative, dt);
    state.angular_velocity.value = integrator(
        state.angular_velocity.value, state.angular_velocity.derivative, dt);
    state.attitude.value =
        integrator(state.attitude.value, state.attitude.derivative, dt);
  }

  void integrate_euler(T dt) {
    integrate(dt, [](const auto &state, const auto &derivative, T dt) {
      return state + derivative * dt;
    });
  }

  Vector3<T> get_position() const { return state.position.value; }

  Matrix3<T> get_attitude() const { return state.attitude.value; }
};

using Rover = RoverT<double>;

} // end namespace Lionheart