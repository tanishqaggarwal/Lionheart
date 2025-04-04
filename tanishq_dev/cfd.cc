#include "tanishq_dev/cfd_lib.h"
#include "tanishq_dev/linalg.h"
#include "tanishq_dev/stl_reader.h"
#include <algorithm>
#include <cstddef>
#include <memory>
#include <string_view>
#include <tuple>

using Lionheart::Vector3f;

/// @brief Tests CFD solution for a "moving lid", which is a classic
/// example for testing CFD codes.
class MovingLidTestcase {
private:
  struct MeshParams {
    /// Mesh discretization
    static constexpr float dx{1e-2}; // m
    static constexpr float dy{1e-2}; // m
    static constexpr float dz{1e-2}; // m
    static constexpr float dt{1e-4}; // s

    /// Mesh size
    static constexpr Lionheart::MeshIndex nX{100};
    static constexpr Lionheart::MeshIndex nY{100};
    static constexpr Lionheart::MeshIndex nZ{100};

    /// Characteristic fluid velocity on Mesh.
    static constexpr float uc{1.0}; // m/s

    /// Stopping criteria for Pressure Poisson equation solver.
    static constexpr float ppe_s_stop = 1e-6;
    static constexpr float ppe_x_stop = 1e-6;
    static constexpr size_t ppe_max_iterations = 10;
  };

  using CFDMesh = Lionheart::CFDMesh<float, MeshParams>;

  static constexpr size_t RAM_LIMIT{32'000'000'000}; // bytes
  static_assert(CFDMesh::size() <= RAM_LIMIT);

  static constexpr size_t N_TIMESTEPS = 100;

public:
  MovingLidTestcase() {
    // Create mesh. Will take a while due to memory allocation.
    CFDMesh mesh;

    // Create "moving lid" boundary condition.
    CFDMesh::Field<Vector3f> bc{{0.0, 0.0, 0.0}};

    for(size_t i = 0; i < N_TIMESTEPS; i++) {
      // Step the solution of the mesh forward in time.
      mesh.step_mesh(bc);
    }
  }
};

int main() {
  MovingLidTestcase testcase;
}