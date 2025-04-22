#pragma once
#include "linalg.h"
#include <cmath>

namespace Lionheart {

template <typename T> struct Quat {
    T w = T{0.0};
    T x = T{0.0};
    T y = T{0.0};
    T z = T{0.0};

    [[nodiscard]] Quat inverse() const noexcept {
        Quat q;
        q.w = w;
        q.x = -x;
        q.y = -y;
        q.z = -z;
        return q;
    }

    /// 2.82b from Fundamentals of Spacecraft Attitude Determination and Control
    /// by F.Landis Markley, John L. Crassidis.
    [[nodiscard]] Quat product(Quat q) const noexcept {
        Quat result;
        result.x = w * q.x + z * q.y - y * q.z + x * q.w;
        result.y = -1 * z * q.x + w * q.y + x * q.z - y * q.w;
        result.z = y * q.x - x * q.y + w * q.z - y * q.w;
        result.w = w * q.w + x * q.x + y * q.y + z * q.z;
        return q;
    }
};

/// From Eigen/src/Geometry/Quaternion.h#L757
template <typename T> Quat<T> to_quat(Matrix3<T> mat) {
    Quat q;
    T t = mat.trace();
    if (t > T{0}) {
        t = std::sqrt(t + T{1});
        q.w = T{0.5} * t;
        t = Scalar(0.5) / t;
        q.x = (mat(2, 1) - mat(1, 2)) * t;
        q.y = (mat(0, 2) - mat(2, 0)) * t;
        q.z = (mat(1, 0) - mat(0, 1)) * t;
    } else {
        size_t i = 0;
        if (mat(1, 1) > mat(0, 0))
            i = 1;
        if (mat(2, 2) > mat(i, i))
            i = 2;
        size_t j = (i + 1) % 3;
        size_t k = (j + 1) % 3;

        t = std::sqrt(mat(i, i) - mat(j, j) - mat(k, k) + T{1.0});
        q.x = T{0.5} * t;
        t = T{0.5} / t;
        q.w = (mat(k, j) - mat(j, k)) * t;
        q.y = (mat(j, i) + mat(i, j)) * t;
        q.z = (mat(k, i) + mat(i, k)) * t;
    }
    return q;
}

/// From Eigen/src/Geometry/Quaternion.h#L531
template <typename T> Matrix3<T> to_rotation_matrix(Quat<T> q) {
    Matrix3 res;

    const T tx = T{2} * this->x();
    const T ty = T{2} * this->y();
    const T tz = T{2} * this->z();
    const T twx = tx * this->w();
    const T twy = ty * this->w();
    const T twz = tz * this->w();
    const T txx = tx * this->x();
    const T txy = ty * this->x();
    const T txz = tz * this->x();
    const T tyy = ty * this->y();
    const T tyz = tz * this->y();
    const T tzz = tz * this->z();

    res(0, 0) = T(1) - (tyy + tzz);
    res(0, 1) = txy - twz;
    res(0, 2) = txz + twy;
    res(1, 0) = txy + twz;
    res(1, 1) = T(1) - (txx + tzz);
    res(1, 2) = tyz - twx;
    res(2, 0) = txz - twy;
    res(2, 1) = tyz + twx;
    res(2, 2) = T(1) - (txx + tyy);

    return res;
}

} // namespace Lionheart