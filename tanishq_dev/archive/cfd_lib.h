#pragma once

#include "tanishq_dev/linalg.h"
#include <algorithm>
#include <cstddef>
#include <memory>
#include <string_view>
#include <tuple>
#include <vector>

namespace Lionheart {

/// Type definition for cell index in the CFD mesh.
using MeshIndex = size_t;

/// @brief Default definition for mesh parameters.
template <typename T> struct DefaultMeshParams {
    /// Mesh discretization
    static constexpr T dx{2e-3}; // m
    static constexpr T dy{2e-3}; // m
    static constexpr T dz{2e-3}; // m
    static constexpr T dt{1e-4}; // s

    /// Mesh size
    static constexpr MeshIndex nX{500};
    static constexpr MeshIndex nY{500};
    static constexpr MeshIndex nZ{500};

    /// Characteristic fluid velocity on Mesh.
    static constexpr T uc{1.0}; // m/s

    /// Stopping criteria for Pressure Poisson equation solver.
    static constexpr T ppe_s_stop = 1e-6;
    static constexpr T ppe_x_stop = 1e-6;
    static constexpr size_t ppe_max_iterations = 10;
};

/// @brief Defines the computational mesh for solution of the Navier-Stokes
/// equations around the body. Assumptions:
/// - Flow is inviscid and incompressible (pretty reasonable for water at low
/// speed)
///
/// The mesh used here is constant-size (not adaptive) and the discretization
/// is chosen to satisfy the CFL condition.
///
/// Under these assumptions the Navier-Stokes equations take the form
///
/// du/dt + u . (nabla u) = -1 / rho nabla^2 P
/// nabla . u = 0
///
/// Really good notes on introductory CFD available at
/// https://www.montana.edu/mowkes/research/source-codes/GuideToCFD.pdf
///
/// @tparam T Floating-point type (expected to be float, or double).
///     Templatizing the parameter helps ensure that the compiler can check
///     for accidental implicit type conversions.
template <typename T = float, typename MeshParams = DefaultMeshParams<T>>
struct CFDMesh {
    // Enforce that mesh has the desired size.
    static_assert(MeshParams::dx * MeshParams::nX >= 1.0);
    static_assert(MeshParams::dy * MeshParams::nY >= 1.0);
    static_assert(MeshParams::dz * MeshParams::nZ >= 1.0);

    // Strongly enforce CFL condition, assuming uc = characteristic velocity.
    static_assert(MeshParams::uc * MeshParams::dt / MeshParams::dx < 0.1);

    /// Storage for quantities defined on the mesh.
    template <typename Tf> class Field {
      private:
        std::vector<Tf> storage;

        /// Notes on Mesh indexing:
        ///
        /// The eventual goal is that fields are indexed such that their entries
        /// are arranged in a space-filling curve (can choose a Morton Z-curve).
        /// This maximizes cache coherence when trying to compute fluxes across
        /// cells in the mesh while minimizing computational cost in
        /// implementation of the indexing scheme.
        ///
        /// For simplicitly, I'll start with a much simpler Cartesian choice.
        /// idx = (((x * nX) + y) * nY) + z
        enum class MeshIndexScheme {
            CARTESIAN,
            MORTON_Z,
        };
        static constexpr MeshIndexScheme idx_scheme =
            MeshIndexScheme::CARTESIAN;

        // Returns an index in the array based on provided coordinates
        static constexpr MeshIndex index(const MeshIndex xi, const MeshIndex yi,
                                         const MeshIndex zi) {
            if constexpr (idx_scheme == MeshIndexScheme::CARTESIAN) {
                return ((xi * MeshParams::nX) + yi * MeshParams::nY) + zi;
            } else if constexpr (idx_scheme == MeshIndexScheme::MORTON_Z) {
                // TODO implement Morton Z curve
                return 0;
            }

            // Impossible, but required for compilation
            return 0;
        }

      protected:
        Tf &val(const MeshIndex xi, const MeshIndex yi, const MeshIndex zi) {
            MeshIndex i = index(xi, yi, zi);
            return storage.at(i);
        }

        const Tf &val(const MeshIndex xi, const MeshIndex yi,
                      const MeshIndex zi) const {
            MeshIndex i = index(xi, yi, zi);
            return storage.at(i);
        }

        Tf val(const MeshIndex i) const { return storage.at(i); }

      public:
        static constexpr MeshIndex nElements =
            MeshParams::nX * MeshParams::nY * MeshParams::nZ;

        Field() : storage(nElements) {}
        explicit Field(Tf initial_value) : storage(nElements, initial_value) {}

        /// Do not provide this as an operator overload; copies should be
        /// explicit since they are expensive.
        void copy(const Field<Tf> &other) { this->storage = other.storage; }

        Tf dot(const Field<Tf> &other) const {
            Tf result = 0;
            for (size_t i = 0; i < nElements; i++) {
                result += this->storage[i] * other.storage[i];
            }
            return result;
        }

        Tf &at(const MeshIndex i) { return storage.at(i); }

        const Tf &at(const MeshIndex i) const { return storage.at(i); }

        Tf &operator()(const MeshIndex xi, const MeshIndex yi,
                       const MeshIndex zi) {
            return val(xi, yi, zi);
        }
        const Tf &operator()(const MeshIndex xi, const MeshIndex yi,
                             const MeshIndex zi) const {
            return val(xi, yi, zi);
        }

        static constexpr size_t size() { return sizeof(Tf) * nElements; }
    };

    /// @brief Class for a scalar field, used by the CFD mesh code.
    ///
    /// @tparam T Type of variable stored in the field (vectors, scalars, etc.)
    class ScalarField : public Field<T> {
      public:
        /// The below functions implement finite-difference partial derivatives
        /// with a central difference.

        T partial_x(const MeshIndex xi, const MeshIndex yi,
                    const MeshIndex zi) const;
        T partial_xx(const MeshIndex xi, const MeshIndex yi,
                     const MeshIndex zi) const;
        T partial_y(const MeshIndex xi, const MeshIndex yi,
                    const MeshIndex zi) const;
        T partial_yy(const MeshIndex xi, const MeshIndex yi,
                     const MeshIndex zi) const;
        T partial_z(const MeshIndex xi, const MeshIndex yi,
                    const MeshIndex zi) const;
        T partial_zz(const MeshIndex xi, const MeshIndex yi,
                     const MeshIndex zi) const;

        /// This vectorized dot-product can be used to perform matrix-vector
        /// operations.
        T dot(const ScalarField &other) const {
            T sum{0.0};
            for (MeshIndex i = 0; i < Field<T>::nElements; i++) {
                sum += this->val(i) * other.val(i);
            }
            return sum;
        }

        T square_magnitude() const {
            T sum{0.0};
            for (MeshIndex i = 0; i < Field<T>::nElements; i++) {
                sum += this->val(i) * this->val(i);
            }
            return sum;
        }

        T magnitude() const { return std::sqrt(this->square_magnitude()); }
    };

    /// @brief Class for a vector field
    struct VectorField {
        ScalarField x;
        ScalarField y;
        ScalarField z;

        static constexpr size_t size() { return 3 * ScalarField::size(); }
    };

    /// Fluid density - chosen to be seawater here
    static constexpr T rho{1030.0}; // kg/m^3

    /// Current time.
    T tval{0.0};

    /// Main "step" method for the CFD.
    ///
    /// The whole point of this evaluation is to get net force and torque on the
    /// body, so that's what this method returns.
    ///
    /// @brief bc Boundary conditions to the flow field (i.e. the velocity along
    /// the surface of the rover body.)
    ///   This field is structured such that its elements are NaN in cells where
    ///   the body is not present, so that we don't impose a boundary condition
    ///   in those cells.
    /// @brief normals Surface normals to the rover body. This field is
    /// structure such that its elements are NaN in cells where the body is not
    /// present.
    /// @brief cg The coordinates of the cell in which the CG of the rover is
    /// located. Used for computing torque.
    ///
    /// @returns A tuple: (force, torque, time).
    ///
    std::tuple<Lionheart::Vector3<T>, Lionheart::Vector3<T>, T>
    step(const Field<Lionheart::Vector3<T>> &bc,
         const Field<Lionheart::Vector3<T>> &normals,
         const Lionheart::Vector3<T> &cg) {
        step_mesh(bc);
        const auto [force, torque] = get_body_force_and_torque(normals, cg);
        return std::make_tuple(force, torque, tval);
    }

    /// Step method for just the CFD mesh.
    ///
    /// @brief bc Boundary conditions to the flow field.
    ///
    void step_mesh(const Field<Lionheart::Vector3<T>> &bc) {
        apply_boundary_condition(bc);
        compute_fluxes();
        compute_pressure();
        compute_next_velocity();
        tval += MeshParams::dt;
        std::swap(velocity, scratchpad.velocity);
    }

    /// Provide pressure so that CFD results can be visualized.
    const ScalarField &get_pressure() const { return pressure; }

  private:
    /// Boundary on the current timestep
    Field<uint8_t> boundary;

    /// Fields that we're solving for.
    ScalarField pressure; // Pressure
    VectorField velocity; // Velocity

    struct Scratchpad {
        VectorField velocity;    // u_next or u_star
        ScalarField divVelocity; // nabla . u_star

        /// Fields for pressure Poisson equation
        ScalarField x;
        ScalarField v;
        ScalarField p;
        ScalarField r;
        ScalarField r0;
        ScalarField s;
        ScalarField t;
        T rho;

        static constexpr size_t size() {
            return VectorField::size() + 8 * ScalarField::size() + sizeof(T);
        }
    } scratchpad;

    /// @brief Initializes the BiCGSTAB method.
    void bicgstab_init();

    /// @brief Implements a step of the BiCGSTAB method.
    ///
    /// @param s_stop Stopping criterion for magnitude of s.
    /// @param x_stop Stopping criterion for magnitude of x.
    /// @return True if the method converged within the criterion, false
    /// otherwise.
    bool bicgstab_step(T s_stop, T x_stop);

  public:
    static constexpr size_t size() {
        return Field<uint8_t>::size() + ScalarField::size() +
               VectorField::size() + Scratchpad::size();
    }

  private:
    /// Executor for any method over the mesh.
    ///
    /// Currently this is just a nested for-loop; later we may take advantage of
    /// GPUs. Can also produce statistics on progress of a step, for example.
    template <typename FunctionT>
    void run_kernel(FunctionT kernel,
                    [[maybe_unused]] const std::string_view debug_info = "") {
        for (MeshIndex xi = 0; xi < MeshParams::nX; xi++) {
            for (MeshIndex yi = 0; yi < MeshParams::nY; yi++) {
                for (MeshIndex zi = 0; zi < MeshParams::nZ; zi++) {
                    kernel(xi, yi, zi);
                }
            }
        }
    }

    /// Executor for any method over fields.
    ///
    /// Currently this is just a for-loop; later we may take advantage of
    /// GPUs. Can also produce statistics on progress of a step, for example.
    template <typename FunctionT>
    void
    run_basic_kernel(FunctionT kernel,
                     [[maybe_unused]] const std::string_view debug_info = "") {
        for (MeshIndex i = 0; i < Field<T>::nElements; i++) {
            kernel(i);
        }
    }

    void apply_boundary_condition(const Field<Lionheart::Vector3<T>> &bc);

    /// Solves the equation
    ///
    /// 1/dt (u_star - u) = - u cdot nabla u
    void compute_fluxes();

    /// Provides the Laplacian for solving the Poisson pressure equation.
    ///
    /// More precisely, since the Laplacian is too large to compute explicitly,
    /// this function computes destination = Laplacian * source.
    void ppe_laplacian_product(const ScalarField &source,
                               ScalarField &destination);

    /// Solves the equation
    ///
    /// nabla^2 p = rho/dt nabla . u_star
    void compute_pressure();

    // Solves the equation
    //
    // 1/dt (u_next - u_star) = -1/rho nabla p
    void compute_next_velocity();

    /// @brief Computes surface integrals of pressure over the body to compute
    /// the net force and torque on the body.
    ///
    /// @param normals Area vectors over the body. If the magnitude of the
    /// vector is zero, so is the contribution to the force/torque.
    /// @param cg Position of CG in the CFD mesh.
    /// @return A tuple containing (force, torque).
    std::tuple<Lionheart::Vector3<T>, Lionheart::Vector3<T>>
    get_body_force_and_torque(const Field<Lionheart::Vector3<T>> &normals,
                              const Lionheart::Vector3<T> &cg) const;
};
} // namespace Lionheart

template <typename T, typename MeshParams>
void swap(typename Lionheart::CFDMesh<T, MeshParams>::ScalarField &lhs,
          typename Lionheart::CFDMesh<T, MeshParams>::ScalarField rhs) {
    std::swap(lhs.storage, rhs.storage);
}

template <typename T, typename MeshParams>
void swap(typename Lionheart::CFDMesh<T, MeshParams>::VectorField &lhs,
          typename Lionheart::CFDMesh<T, MeshParams>::VectorField rhs) {
    std::swap(lhs.x, rhs.x);
    std::swap(lhs.y, rhs.y);
    std::swap(lhs.z, rhs.z);
}

#include "tanishq_dev/cfd_lib_impl.h"