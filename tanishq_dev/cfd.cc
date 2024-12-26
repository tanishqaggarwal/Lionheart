#include "tanishq_dev/cfd_lib.h"
#include "tanishq_dev/linalg.h"
#include "tanishq_dev/stl_reader.h"
#include <algorithm>
#include <cstddef>
#include <memory>
#include <string_view>
#include <tuple>

template<typename T>
struct TestMeshParams {
  /// Mesh discretization
  static constexpr T dx{2e-3}; // m
  static constexpr T dy{2e-3}; // m
  static constexpr T dz{2e-3}; // m
  static constexpr T dt{1e-4}; // s

  /// Mesh size
  static constexpr Lionheart::MeshIndex nX{500};
  static constexpr Lionheart::MeshIndex nY{500};
  static constexpr Lionheart::MeshIndex nZ{500};

  /// Characteristic fluid velocity on Mesh.
  static constexpr T uc{1.0}; // m/s

  /// Stopping criteria for Pressure Poisson equation solver.
  static constexpr T ppe_s_stop = 1e-6;
  static constexpr T ppe_x_stop = 1e-6;
  static constexpr size_t ppe_max_iterations = 10;
};

using CFDMesh = Lionheart::CFDMesh<float, TestMeshParams<float>>;
using Lionheart::Vector3f;

static constexpr size_t RAM_LIMIT{32'000'000'000}; // bytes
static_assert(CFDMesh::size() <= RAM_LIMIT);

int main() {
  // Create mesh. Will take a while due to memory allocation.
  CFDMesh mesh;

  // TODO populate inputs to CFD solution from STL.
  CFDMesh::Field<Vector3f> bc;
  CFDMesh::Field<Vector3f> normals;
  Lionheart::Vector3f cg{};

  // TODO perform and then assess dynamics.
  mesh.step(bc, normals, cg);
}