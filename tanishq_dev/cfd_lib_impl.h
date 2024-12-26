
namespace Lionheart {
template <typename T, typename MeshParams>
T CFDMesh<T, MeshParams>::ScalarField::partial_x(const MeshIndex xi,
                                                 const MeshIndex yi,
                                                 const MeshIndex zi) const {
  if (xi >= MeshParams::nX - 1 || xi == 0)
    return T{0.0};
  return 0.5 * (Field<T>::val(xi + 1, yi, zi) - Field<T>::val(xi - 1, yi, zi)) /
         MeshParams::dx;
}

template <typename T, typename MeshParams>
T CFDMesh<T, MeshParams>::ScalarField::partial_xx(const MeshIndex xi,
                                                  const MeshIndex yi,
                                                  const MeshIndex zi) const {
  if (xi >= MeshParams::nX - 1 || xi == 0)
    return T{0.0};
  return (Field<T>::val(xi + 1, yi, zi) - 2 * Field<T>::val(xi, yi, zi) +
          Field<T>::val(xi - 1, yi, zi)) /
         (MeshParams::dx * MeshParams::dx);
}

template <typename T, typename MeshParams>
T CFDMesh<T, MeshParams>::ScalarField::partial_y(const MeshIndex xi,
                                                 const MeshIndex yi,
                                                 const MeshIndex zi) const {
  if (yi >= MeshParams::nY - 1 || yi == 0)
    return T{0.0};
  return 0.5 * (Field<T>::val(xi, yi + 1, zi) - Field<T>::val(xi, yi - 1, zi)) /
         MeshParams::dy;
}

template <typename T, typename MeshParams>
T CFDMesh<T, MeshParams>::ScalarField::partial_yy(const MeshIndex xi,
                                                  const MeshIndex yi,
                                                  const MeshIndex zi) const {
  if (yi >= MeshParams::nY - 1 || yi == 0)
    return T{0.0};
  return (Field<T>::val(xi, yi + 1, zi) - 2 * Field<T>::val(xi, yi, zi) +
          Field<T>::val(xi, yi - 1, zi)) /
         (MeshParams::dy * MeshParams::dy);
}

template <typename T, typename MeshParams>
T CFDMesh<T, MeshParams>::ScalarField::partial_z(const MeshIndex xi,
                                                 const MeshIndex yi,
                                                 const MeshIndex zi) const {
  if (zi >= MeshParams::nZ - 1 || zi == 0)
    return T{0.0};
  return 0.5 * (Field<T>::val(xi, yi, zi + 1) - Field<T>::val(xi, yi, zi - 1)) /
         MeshParams::dz;
}

template <typename T, typename MeshParams>
T CFDMesh<T, MeshParams>::ScalarField::partial_zz(const MeshIndex xi,
                                                  const MeshIndex yi,
                                                  const MeshIndex zi) const {
  if (zi >= MeshParams::nZ - 1 || zi == 0)
    return T{0.0};
  return (Field<T>::val(xi, yi, zi + 1) - 2 * Field<T>::val(xi, yi, zi) +
          Field<T>::val(xi, yi, zi - 1)) /
         (MeshParams::dz * MeshParams::dz);
}

template <typename T, typename MeshParams>
void CFDMesh<T, MeshParams>::bicgstab_init()
{
  const auto& s = this->scratchpad;
  const auto& b = s.velocity;

  s.x.copy(this->pressure);

  // Compute r0 = b - A x_0
  ppe_laplacian_product(s.x, s.r0);
  run_kernel([&s, &b](MeshIndex xi, MeshIndex yi, MeshIndex zi) {
    s.r0(xi, yi, zi) -= b(xi, yi, zi);
    s.r0(xi, yi, zi) *= -1;
  });

  // Compute rho0
  s.rho = s.r0.square_magnitude();

  // Compute p0
  s.p.copy(s.r0);
}

template <typename T, typename MeshParams>
bool CFDMesh<T, MeshParams>::bicgstab_step(T s_stop, T x_stop)
{
  const auto& s = this->scratchpad;

  // v = A p_(i-1)
  ppe_laplacian_product(s.p, s.v);

  const auto alpha = s.rho / (s.r0.dot(s.v));

  // s = r_(i-1) - alpha v_i
  run_basic_kernel([&s, alpha](MeshIndex i) {
    s.at(i) = s.r.at(i) - alpha * s.v.at(i);
  });

  // Quit and return x_i = h if s is small enough
  if (s.s.square_magnitude() < s_stop)
  {
    // h = x - alpha * p
    run_basic_kernel([&s, alpha](MeshIndex i) {
      s.x.at(i) = s.x.at(i) + alpha * s.p.at(i);
    });
    return true;
  }

  // t = A s
  ppe_laplacian_product(s.s, s.t);

  // x = h + s omega
  const auto omega = s.t.dot(s.s) / s.t.square_magnitude();
  run_basic_kernel([&s, alpha, omega](MeshIndex i) {
    const auto hi = s.x.at(i) + alpha * s.p.at(i);
    s.x.at(i) = hi + omega * s.s.at(i);
  });

  // Quit if x is small enough
  if (s.x.square_magnitude() < s_stop)
  {
    return true;
  }

  // r = s - omega t  
  run_basic_kernel([&s, omega](MeshIndex i) {
    s.r.at(i) = s.s.at(i) - omega * s.t.at(i);
  });

  const auto rho_prev = s.rho;
  s.rho = s.r0.dot(s.r);
  const auto beta = (s.rho/rho_prev) * (alpha/omega);

  // p_i = r_i + beta(p_(i-1) - omega v)
  run_basic_kernel([&s, omega, beta](MeshIndex i) {
    s.p.at(i) = s.r.at(i) - beta * (s.p.at(i) - omega * s.v.at(i));
  });

  return false;
}

template <typename T, typename MeshParams>
void CFDMesh<T, MeshParams>::apply_boundary_condition(
    const CFDMesh<T, MeshParams>::Field<Vector3<T>> &bc) {
  auto &boundary = this->boundary;
  auto &u = this->velocity.x;
  auto &v = this->velocity.y;
  auto &w = this->velocity.z;
  run_kernel([&boundary, &bc, &u, &v, &w](size_t xi, size_t yi, size_t zi) {
    const Vector3<T> uB = bc(xi, yi, zi);
    if (!uB.isnan()) {
      u(xi, yi, zi) = uB.x;
      v(xi, yi, zi) = uB.y;
      w(xi, yi, zi) = uB.z;
      boundary(xi, yi, zi) = 0;
    } else {
      boundary(xi, yi, zi) = 1;
    }
  });
}

template <typename T, typename MeshParams>
void CFDMesh<T, MeshParams>::compute_fluxes() {
  const auto &boundary = this->boundary;
  const auto &u = this->velocity.x;
  const auto &v = this->velocity.y;
  const auto &w = this->velocity.z;
  auto &uStar = this->scratchpad.velocity.x;
  auto &vStar = this->scratchpad.velocity.y;
  auto &wStar = this->scratchpad.velocity.z;

  run_kernel([&boundary, &u, &v, &w, &uStar, &vStar,
              &wStar](size_t xi, size_t yi, size_t zi) {
    if (boundary(xi, yi, zi) == 1) {
      uStar(xi, yi, zi) = 0.0;
      vStar(xi, yi, zi) = 0.0;
      wStar(xi, yi, zi) = 0.0;
      return;
    }
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

    uStar(xi, yi, zi) =
        ui - MeshParams::dt * (ui * ui_x + vi * ui_y + wi * ui_z);
    vStar(xi, yi, zi) =
        vi - MeshParams::dt * (ui * vi_x + vi * vi_y + wi * vi_z);
    wStar(xi, yi, zi) =
        wi - MeshParams::dt * (ui * wi_x + vi * wi_y + wi * wi_z);
  });
}

template <typename T, typename MeshParams>
void CFDMesh<T, MeshParams>::ppe_laplacian_product(
    const CFDMesh<T, MeshParams>::ScalarField &source,
    CFDMesh<T, MeshParams>::ScalarField &destination) {
  // TODO correctly compute Laplacian, given bounds and stuff.
}

template <typename T, typename MeshParams>
void CFDMesh<T, MeshParams>::compute_pressure() {
  // Store rho/dt nabla . u_star
  const auto &boundary = this->boundary;
  const auto &uStar = this->scratchpad.velocity.x;
  const auto &vStar = this->scratchpad.velocity.y;
  const auto &wStar = this->scratchpad.velocity.z;
  auto &div = this->scratchpad.divVelocity;
  run_kernel([&boundary, &uStar, &vStar, &wStar, &div](size_t xi, size_t yi,
                                                       size_t zi) {
    if (boundary(xi, yi, zi) == 1) {
      div(xi, yi, zi) = 0.0;
      return;
    }
    div(xi, yi, zi) = uStar.partial_x(xi, yi, zi) +
                      vStar.partial_y(xi, yi, zi) + wStar.partial_z(xi, yi, zi);
  });

  // I think what we want to do here is use
  // https://en.wikipedia.org/wiki/Biconjugate_gradient_stabilized_method
  // which is a low-storage method for iteratively finding the solution to
  // pressure.
  // TODO
}

template <typename T, typename MeshParams>
void CFDMesh<T, MeshParams>::compute_next_velocity() {
  const auto &boundary = this->boundary;
  const auto &P = this->pressure;
  const auto &uStar = this->scratchpad.velocity.x;
  const auto &vStar = this->scratchpad.velocity.y;
  const auto &wStar = this->scratchpad.velocity.z;
  auto &uNext = this->scratchpad.velocity.x;
  auto &vNext = this->scratchpad.velocity.y;
  auto &wNext = this->scratchpad.velocity.z;

  run_kernel([&boundary, &P, &uStar, &vStar, &wStar, &uNext, &vNext,
              &wNext](size_t xi, size_t yi, size_t zi) {
    if (boundary(xi, yi, zi) == 1) {
      uNext(xi, yi, zi) = 0.0;
      vNext(xi, yi, zi) = 0.0;
      wNext(xi, yi, zi) = 0.0;
      return;
    }
    uNext(xi, yi, zi) =
        uStar(xi, yi, zi) - (MeshParams::dt / rho) * P.partial_x(xi, yi, zi);
    vNext(xi, yi, zi) =
        vStar(xi, yi, zi) - (MeshParams::dt / rho) * P.partial_y(xi, yi, zi);
    wNext(xi, yi, zi) =
        wStar(xi, yi, zi) - (MeshParams::dt / rho) * P.partial_z(xi, yi, zi);
  });
}

/// @brief Computes surface integrals of pressure over the body to compute
/// the net force and torque on the body.
///
/// @param normals Area vectors over the body. If the magnitude of the vector
/// is zero, so is the contribution to the force/torque.
/// @param cg Position of CG in the CFD mesh.
/// @return A tuple containing (force, torque).
template <typename T, typename MeshParams>
std::tuple<Vector3<T>, Vector3<T>>
CFDMesh<T, MeshParams>::get_body_force_and_torque(
    const CFDMesh<T, MeshParams>::Field<Vector3<T>> &normals,
    const Vector3<T> &cg) const {
  Vector3<T> force, torque;

  for (size_t xi = 0; xi < MeshParams::nX; xi++) {
    for (size_t yi = 0; yi < MeshParams::nY; yi++) {
      for (size_t zi = 0; zi < MeshParams::nZ; zi++) {
        const auto dF = normals(xi, yi, zi) * this->pressure(xi, yi, zi);
        force += dF;
        torque += (Vector3<T>{xi * MeshParams::dx, yi * MeshParams::dy,
                              zi * MeshParams::dz} -
                   cg)
                      .cross(dF);
      }
    }
  }
  return std::make_tuple(force, torque);
}

} // end namespace Lionheart