/// Implements a minimal linear algebra library because it is extremely tedious
// to compile Eigen for embedded platforms through Bazel.
//
// @author Tanishq Aggarwal
// @date 2024-12-13

#pragma once
#include <cmath>

namespace Lionheart
{

template<typename T>
struct Vector3
{
    T x = 0.0;
    T y = 0.0;
    T z = 0.0;

    [[nodiscard]] T dot(const Vector3<T>& other) const
    {
        return this->x * other.x + this->y * other.y + this->z * other.z;
    }
    [[nodiscard]] T norm() const
    {
        return std::sqrt(x*x + y*y + z*z);
    }

    Vector3 operator+=(const Vector3& other)
    {
        *this = *this + other;
        return *this;
    }

    Vector3 operator-=(const Vector3& other)
    {
        *this = *this - other;
        return *this;
    }
};

Vector3 operator+(const Vector3& v1, const Vector3& v2)
{
    return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

Vector3 operator-(const Vector3& v1, const Vector3& v2)
{
    return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}

Vector3 operator*(const Vector3& v, const double c)
{
    return {v.x * c, v.y * c, v.z * c};
}

Vector3 operator/(const Vector3& v, const double c)
{
    return {v.x / c, v.y / c, v.z / c};
}

} // namespace Lionheart