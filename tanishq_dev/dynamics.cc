#include <array>
#include <cmath>

#include "tanishq_dev/linalg.h"

namespace Lionheart
{

/// @brief Implements 6DOF dynamics for rover.
template<typename T>
struct RoverT
{
    /// Density of seawater in the operating region
    static constexpr T water_density { 1035.0 }; // kg / m^3

    /// Inertial acceleration due to gravity
    static constexpr Vector3<T> accel_gravity {0, 0, 9.81}; // m/s^2

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
    integrated_state_t<Vector3<T>> angvel;

    /// DCM translating vectors from body frame to inertial frame.
    integrated_state_t<Matrix3<T>> attitude;

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

    std::tuple<Matrix3<T>, T> added_mass() const
    {
        // Compute the "effective" mass and MOI due to motion through water.
        // TODO
        // Assumptions:
        // - Water is inviscid and incompressible
        // - Make some assumptions about body geometry

        return std::make_tuple(Matrix3<T>{}, T{0.0});
    }

    void update(const std::array<T, N_THRUSTERS>& thrusts)
    {
        // Get "added mass" effects.
        const auto [moi_added, m_added] = added_mass();

        // Get net thrust force in body frame.
        Vector3<T> net_thrust_force;
        for(size_t i = 0; i < N_THRUSTERS; i++)
        {
            net_thrust_force += thrust_vectors[i] * thrusts[i];
        }

        // Get net thrust torque in body frame.
        Vector3<T> net_thrust_torque;
        for(size_t i = 0; i < N_THRUSTERS; i++)
        {
            net_thrust_torque += thrust_positions[i].cross(thrust_vectors[i] * thrusts[i]);
        }

        // Get net buoyancy
        Vector3<T> buoyancy_force = accel_gravity * water_density * volume * Vector3<T>{T{0}, T{0}, T{1}}; // Inertial frame
        Vector3<T> buoyancy_torque = cb.cross(attitude.value.inverse() * buoyancy_force); // Body frame

        // TODO compute drag force, current disturbances

        // Aggregate torques and forces
        Vector3<T> net_torque_body = buoyancy_torque + net_thrust_torque;
        Vector3<T> net_force_inertial = buoyancy_force + attitude.value * net_thrust_force;
        
        // Compute equations of motion
        position.derivative = velocity.value;
        velocity.derivative = net_force_inertial / (mass + m_added);
        angvel.derivative = moi_inv * ( net_torque_body - angvel.value.cross(moi * angvel.value) );

        const auto& w = angvel.value;
        attitude.derivative = attitude.value * Matrix3<T>{{0, -w.z, w.y}, {w.z, 0, -w.x}, {-w.y, w.x, 0}};
    }
};

using Rover = RoverT<double>;

} // end namespace Lionheart