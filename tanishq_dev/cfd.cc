#include "tanishq_dev/cfd_lib.h"
#include "tanishq_dev/linalg.h"
#include "tanishq_dev/stl_reader.h"
#include <algorithm>
#include <cstddef>
#include <memory>
#include <string_view>
#include <tuple>

using CFDMesh = Lionheart::CFDMesh<float, Lionheart::DefaultMeshParams<float>>;
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