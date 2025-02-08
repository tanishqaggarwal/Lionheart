#pragma once

#include <array>
#include <cmath>

#include "tanishq_dev/linalg.h"

namespace Lionheart
{

/// @brief Implements 6DOF dynamics for rover.
template<typename T>
struct RoverT
{
    // Internal state of system
    template<typename Quantity>
    struct integrated_state_t
    {
        Quantity value;
        Quantity derivative;
    };

    /// Position and velocity of CG, inertial frame.
    integrated_state_t<Vector3<T>> position;
    integrated_state_t<Vector3<T>> velocity;

    /// Angular velocity of body w.r.t. CG, body frame.
    integrated_state_t<Vector3<T>> angular_velocity;

    /// DCM translating vectors from body frame to inertial frame.
    integrated_state_t<Matrix3<T>> attitude;

    struct config_t {
        /// Density of seawater in the operating region
        static constexpr T water_density { 1035.0 }; // kg / m^3

        /// Inertial acceleration due to gravity
        static constexpr Vector3<T> accel_gravity {0, 0, 9.81}; // m/s^2

        /// Total mass of rover (accounting for internal water storage as well.)
        T mass {0.0};

        /// Total dry volume of rover.
        T volume {0.0};

        /// Moment of inertia (and its inverse) in body frame.
        Matrix3<T> moi;
        Matrix3<T> moi_inv;

        /// Location of center of buoyancy relative to CG in body frame.
        Vector3<T> cb;

        /// Positions and directions of thrust vectors, in body frame.
        static constexpr size_t N_THRUSTERS { 5 };
        std::array<Vector3<T>, N_THRUSTERS> thrust_positions;
        std::array<Vector3<T>, N_THRUSTERS> thrust_vectors;
    } config;

    explicit RoverT(const config_t& config_) : config(config_)
    {}

    // Compute the "effective" mass and MOI due to motion through water.
    // TODO
    // Assumptions:
    // - Water is inviscid and incompressible
    // - Make some assumptions about body geometry
    std::tuple<Matrix3<T>, T> added_mass() const
    {
        return std::make_tuple(Matrix3<T>{}, T{0.0});
    }

    void update(const std::array<T, config_t::N_THRUSTERS>& thrusts)
    {
        // Get "added mass" effects.
        const auto [moi_added, m_added] = added_mass();

        // Get net thrust force in body frame.
        Vector3<T> net_thrust_force;
        for(size_t i = 0; i < config_t::N_THRUSTERS; i++)
        {
            net_thrust_force += config.thrust_vectors[i] * thrusts[i];
        }

        // Get net thrust torque in body frame.
        Vector3<T> net_thrust_torque;
        for(size_t i = 0; i < config_t::N_THRUSTERS; i++)
        {
            net_thrust_torque += config.thrust_positions[i].cross(config.thrust_vectors[i] * thrusts[i]);
        }

        // Get net buoyancy
        Vector3<T> buoyancy_force = -config_t::water_density * config.volume * config_t::accel_gravity; // Inertial frame
        Vector3<T> buoyancy_torque = config.cb.cross(attitude.value.inverse() * buoyancy_force); // Body frame

        // TODO compute drag force, current disturbances

        // Aggregate torques and forces
        Vector3<T> net_torque_body = buoyancy_torque + net_thrust_torque;
        Vector3<T> net_force_inertial = buoyancy_force + attitude.value * net_thrust_force;
        
        // Compute equations of motion
        position.derivative = velocity.value;
        velocity.derivative = net_force_inertial / (config.mass + m_added);
        angular_velocity.derivative = config.moi_inv * ( net_torque_body - angular_velocity.value.cross(config.moi * angular_velocity.value) );

        const auto& w = angular_velocity.value;
        attitude.derivative = attitude.value * Matrix3<T>{{0, -w.z, w.y}, {w.z, 0, -w.x}, {-w.y, w.x, 0}};
    }
};

using Rover = RoverT<double>;

} // end namespace Lionheart