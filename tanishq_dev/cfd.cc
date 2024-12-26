#include "tanishq_dev/cfd_lib.h"
#include "tanishq_dev/linalg.h"
#include "tanishq_dev/stl_reader.h"
#include <algorithm>
#include <cstddef>
#include <memory>
#include <string_view>
#include <tuple>

using Lionheart::Vector3f;

/// @brief Tests CFD solution for a rover that is completely still.
class StillRoverTestcase {
private:
  struct MeshParams {
    /// Mesh discretization
    static constexpr float dx{2e-3}; // m
    static constexpr float dy{2e-3}; // m
    static constexpr float dz{2e-3}; // m
    static constexpr float dt{1e-4}; // s

    /// Mesh size
    static constexpr Lionheart::MeshIndex nX{500};
    static constexpr Lionheart::MeshIndex nY{500};
    static constexpr Lionheart::MeshIndex nZ{500};

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

public:
  StillRoverTestcase() {
    // Create mesh. Will take a while due to memory allocation.
    CFDMesh mesh;

    // Populate inputs to CFD solution from STL.
    CFDMesh::Field<Vector3f> bc; // This should be zero; the fluid is still.
    CFDMesh::Field<Vector3f> normals; // TODO

    // Put rover CG at center of mesh
    Lionheart::Vector3f cg{};
    cg(0) = MeshParams::nX/2 * MeshParams::dx;
    cg(1) = MeshParams::nY/2 * MeshParams::dy;
    cg(2) = MeshParams::nZ/2 * MeshParams::dz;

    // Perform and then assess dynamics.
    mesh.step(bc, normals, cg);
  }
};

int main() {
  StillRoverTestcase testcase;
}