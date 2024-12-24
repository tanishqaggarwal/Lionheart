#include "tanishq_dev/cfd/stl_reader.h"
#include <algorithm>
#include <cstddef>
#include <memory>
#include <string_view>
#include <tuple>

/// Not using this yet; will assume the Rover is a cube, for simplicity.
using StlMesh = stl_reader::StlMesh<float, unsigned int>;

/// Type definition for index in the CFD mesh.
using Index = size_t;

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
template <typename T> struct CFDMeshT {
  /// Mesh discretization
  static constexpr T dx{2e-3}; // m
  static constexpr T dy{2e-3}; // m
  static constexpr T dz{2e-3}; // m
  static constexpr T dt{1e-4}; // s

  /// Mesh size
  static constexpr Index nX{500};
  static constexpr Index nY{500};
  static constexpr Index nZ{500};
  static constexpr Index nElements = nX * nY * nZ;

  // Enforce that mesh can fit in RAM.
  static constexpr T RAM_LIMIT{1e10}; // bytes
  static_assert(nElements * sizeof(T) <= RAM_LIMIT);

  // Enforce that mesh has the desired size.
  static_assert(dx * nX >= 1.0);
  static_assert(dy * nY >= 1.0);
  static_assert(dz * nZ >= 1.0);

  // Strongly enforce CFL condition, assuming uc = characteristic velocity.
  static constexpr T uc{1.0}; // m/s
  static_assert(uc * dt / dx < 0.1);

  /// @brief Class for a mathematical field, used by the CFD mesh code.
  ///
  /// @tparam T Type of variable stored in the field (vectors, scalars, etc.)
  struct Field {
    /// Notes on indexing:
    ///
    /// The eventual goal is that fields are indexed such that their entries
    /// are arranged in a space-filling curve (can choose a Morton Z-curve).
    /// This maximizes cache coherence when trying to compute fluxes across
    /// cells in the mesh while minimizing computational cost in
    /// implementation of the indexing scheme.
    ///
    /// For simplicitly, I'll start with a much simpler Cartesian choice.
    /// idx = (((x * nX) + y) * nY) + z
    enum class IndexScheme {
      CARTESIAN,
      MORTON_Z,
    };
    static constexpr IndexScheme idx_scheme = IndexScheme::CARTESIAN;

    static constexpr Index index(const Index xi, const Index yi,
                                 const Index zi) {
      if constexpr (idx_scheme == IndexScheme::CARTESIAN) {
        return ((xi * nX) + yi * nY) + zi;
      } else if constexpr (idx_scheme == IndexScheme::MORTON_Z) {
        //// TODO
        return 0;
      }

      /// Impossible, but required for compilation
      return 0;
    }

  private:
    std::vector<T> storage;

    T &val(const Index xi, const Index yi, const Index zi) {
      Index i = index(xi, yi, zi);
      return storage.at(i);
    }

    const T &val(const Index xi, const Index yi, const Index zi) const {
      Index i = index(xi, yi, zi);
      return storage.at(i);
    }

  public:
    explicit Field(Index size) : storage(size) {}

    T &operator()(const Index xi, const Index yi, const Index zi) {
      return val(xi, yi, zi);
    }
    const T &operator()(const Index xi, const Index yi, const Index zi) const {
      return val(xi, yi, zi);
    }

    T partial_x(const Index xi, const Index yi, const Index zi) const {
      if (xi >= nX - 1 || xi == 0)
        return T{0.0};
      return (0.5 / dx) * (val(xi + 1, yi, zi) - val(xi - 1, yi, zi));
    }

    T partial_xx(const Index xi, const Index yi, const Index zi) const {
      if (xi >= nX - 1 || xi == 0)
        return T{0.0};
      return (1 / (dx * dx)) *
             (val(xi + 1, yi, zi) - 2 * val(xi, yi, zi) + val(xi - 1, yi, zi));
    }

    T partial_y(const Index xi, const Index yi, const Index zi) const {
      if (yi >= nY - 1 || yi == 0)
        return T{0.0};
      return (0.5 / dy) * (val(xi, yi + 1, zi) - val(xi, yi - 1, zi));
    }

    T partial_yy(const Index xi, const Index yi, const Index zi) const {
      if (yi >= nY - 1 || yi == 0)
        return T{0.0};
      return (1 / (dy * dy)) *
             (val(xi, yi + 1, zi) - 2 * val(xi, yi, zi) + val(xi, yi - 1, zi));
    }

    T partial_z(const Index xi, const Index yi, const Index zi) const {
      if (zi >= nZ - 1 || zi == 0)
        return T{0.0};
      return (0.5 / dz) * (val(xi, yi, zi + 1) - val(xi, yi, zi - 1));
    }

    T partial_zz(const Index xi, const Index yi, const Index zi) const {
      if (zi >= nZ - 1 || zi == 0)
        return T{0.0};
      return (1 / (dz * dz)) *
             (val(xi, yi, zi + 1) - 2 * val(xi, yi, zi) + val(xi, yi, zi - 1));
    }
  };

  /// @brief Class for a vector field
  struct VectorField {
    Field x;
    Field y;
    Field z;

    explicit VectorField(Index nElements)
        : x(nElements), y(nElements), z(nElements) {}
  };

  /// Fields that we're solving for.
  Field pressure{nElements};       // Pressure
  VectorField velocity{nElements}; // Velocity

  /// Fluid density - chosen to be seawater here
  static constexpr T rho{1030.0}; // kg/m^3

  /// Current time.
  T tval{0.0};

  /// Main "step" method for the CFD.
  T step(const VectorField &bc) {
    apply_boundary_condition(bc);
    compute_fluxes();
    compute_pressure();
    compute_next_velocity();
    tval += dt;
    std::swap(velocity, velocityNext);
    return tval;
  }

private:
  /// Intermediate scratchpads for solution of fields.
  VectorField velocityStar{nElements}; // u_star
  VectorField velocityNext{nElements}; // u_next

  struct PressureCalculationFields {
    Field divVelocityStar{nElements}; // nabla . u_star

    /// TODO add more fields
  } pressure_scratchpad;

  /// Executor for any method over the mesh.
  ///
  /// Currently this is just a nested for-loop; later we may take advantage of
  /// GPUs. Can also produce statistics on progress of a step, for example.
  template <typename FunctionT>
  void run_kernel(FunctionT kernel,
                  [[maybe_unused]] const std::string_view debug_info = "") {
    for (Index xi = 0; xi < nX; xi++) {
      for (Index yi = 0; yi < nY; yi++) {
        for (Index zi = 0; zi < nZ; zi++) {
          kernel(xi, yi, zi);
        }
      }
    }
  }

  void apply_boundary_condition(const VectorField &bc) {
    const Field &uB = bc.x;
    const Field &vB = bc.y;
    const Field &wB = bc.z;
    Field &u = this->velocity.x;
    Field &v = this->velocity.y;
    Field &w = this->velocity.z;
    run_kernel([&uB, &vB, &wB, &u, &v, &w](size_t xi, size_t yi, size_t zi) {
      const T uBi = uB(xi, yi, zi);
      const T vBi = vB(xi, yi, zi);
      const T wBi = wB(xi, yi, zi);
      if (!std::isnan(uBi)) {
        u(xi, yi, zi) = uBi;
      }
      if (!std::isnan(vBi)) {
        v(xi, yi, zi) = vBi;
      }
      if (!std::isnan(wBi)) {
        w(xi, yi, zi) = wBi;
      }
    });
  }

  void compute_fluxes() {
    // Solves the equation
    //
    // 1/dt (u_star - u) = - u cdot nabla u

    // Create shortcuts
    const Field &u = this->velocity.x;
    const Field &v = this->velocity.y;
    const Field &w = this->velocity.z;
    Field &uStar = this->velocityStar.x;
    Field &vStar = this->velocityStar.y;
    Field &wStar = this->velocityStar.z;

    run_kernel(
        [&u, &v, &w, &uStar, &vStar, &wStar](size_t xi, size_t yi, size_t zi) {
          const T ui = u(xi, yi, zi);
          const T vi = v(xi, yi, zi);
          const T wi = w(xi, yi, zi);
          const T ui_x = u.partial_x(xi, yi, zi);
          const T ui_y = u.partial_y(xi, yi, zi);
          const T ui_z = u.partial_z(xi, yi, zi);
          const T vi_x = v.partial_x(xi, yi, zi);
          const T vi_y = v.partial_y(xi, yi, zi);
          const T vi_z = v.partial_z(xi, yi, zi);
          const T wi_x = w.partial_x(xi, yi, zi);
          const T wi_y = w.partial_y(xi, yi, zi);
          const T wi_z = w.partial_z(xi, yi, zi);

          uStar(xi, yi, zi) = ui - dt * (ui * ui_x + vi * ui_y + wi * ui_z);
          vStar(xi, yi, zi) = vi - dt * (ui * vi_x + vi * vi_y + wi * vi_z);
          wStar(xi, yi, zi) = wi - dt * (ui * wi_x + vi * wi_y + wi * wi_z);
        });
  }

  /// Provides the Laplacian for solving the PPE equation.
  ///
  /// More precisely, since the Laplacian is too large to compute explicitly,
  /// this function computes destination = Laplacian * source.
  void ppe_laplacian_product(const Field &source, Field &destination) {
    /// TODO correctly compute Laplacian, given bounds and stuff.
  }

  void compute_pressure() {
    // Solves the equation
    //
    // nabla^2 p = rho/dt nabla . u_star

    /// Store rho/dt nabla . u_star
    const Field &uStar = this->velocityStar.x;
    const Field &vStar = this->velocityStar.y;
    const Field &wStar = this->velocityStar.z;
    Field &div = this->pressure_scratchpad.divVelocityStar;
    run_kernel([&uStar, &vStar, &wStar, &div](size_t xi, size_t yi, size_t zi) {
      div(xi, yi, zi) = uStar.partial_x(xi, yi, zi) +
                        vStar.partial_y(xi, yi, zi) +
                        wStar.partial_z(xi, yi, zi);
    });

    /// I think what we want to do here is use
    /// https://en.wikipedia.org/wiki/Biconjugate_gradient_stabilized_method
    /// which is a low-storage method for iteratively finding the solution to
    /// p.
  }

  void compute_next_velocity() {
    // Solves the equation
    //
    // 1/dt (u_next - u_star) = -1/rho nabla p;

    const Field &P = this->pressure;
    const Field &uStar = this->velocityStar.x;
    const Field &vStar = this->velocityStar.y;
    const Field &wStar = this->velocityStar.z;
    Field &uNext = this->velocityNext.x;
    Field &vNext = this->velocityNext.y;
    Field &wNext = this->velocityNext.z;

    run_kernel([&P, &uStar, &vStar, &wStar, &uNext, &vNext,
                &wNext](size_t xi, size_t yi, size_t zi) {
      uNext(xi, yi, zi) =
          uStar(xi, yi, zi) - (dt / rho) * P.partial_x(xi, yi, zi);
      vNext(xi, yi, zi) =
          vStar(xi, yi, zi) - (dt / rho) * P.partial_y(xi, yi, zi);
      wNext(xi, yi, zi) =
          wStar(xi, yi, zi) - (dt / rho) * P.partial_z(xi, yi, zi);
    });
  }
};

template <typename T>
void swap(typename CFDMeshT<T>::Field &lhs, typename CFDMeshT<T>::Field rhs) {
  std::swap(lhs.storage, rhs.storage);
}

template <typename T>
void swap(typename CFDMeshT<T>::VectorField &lhs,
          typename CFDMeshT<T>::VectorField rhs) {
  std::swap(lhs.x, rhs.x);
  std::swap(lhs.y, rhs.y);
  std::swap(lhs.z, rhs.z);
}

using CFDMesh = CFDMeshT<float>;

int main() {
  CFDMesh mesh;

  CFDMesh::VectorField bc{CFDMesh::nElements};
  mesh.step(bc);
}