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
    T x = T{0.0};
    T y = T{0.0};
    T z = T{0.0};

    [[nodiscard]] T dot(const Vector3<T>& other) const
    {
        return this->x * other.x + this->y * other.y + this->z * other.z;
    }

    [[nodiscard]] Vector3<T> cross(const Vector3<T>& other) const
    {
        Vector3 result;
        result.x = this->y * other.z - other.y * this->z;
        result.y = this->z * other.x - other.z * this->x;
        result.z = this->x * other.y - other.x * this->y;
        return result;
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

    void clear()
    {
        x = T{0.0};
        y = T{0.0};
        z = T{0.0};
    }
};

template<typename T>
Vector3<T> operator+(const Vector3<T>& v1, const Vector3<T>& v2)
{
    return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

template<typename T>
Vector3<T> operator-(const Vector3<T>& v1, const Vector3<T>& v2)
{
    return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}

template<typename T>
Vector3<T> operator*(const Vector3<T>& v, const T c)
{
    return {v.x * c, v.y * c, v.z * c};
}

template<typename T>
Vector3<T> operator*(const T c, const Vector3<T>& v)
{
    return v * c;
}

template<typename T>
Vector3<T> operator/(const Vector3<T>& v, const T c)
{
    return {v.x / c, v.y / c, v.z / c};
}

template<typename T>
struct Matrix3
{
    Vector3<T> r1;
    Vector3<T> r2;
    Vector3<T> r3;

    Matrix3 operator+=(const Matrix3& other)
    {
        *this = *this + other;
        return *this;
    }

    Matrix3 operator-=(const Matrix3& other)
    {
        *this = *this - other;
        return *this;
    }

    [[nodiscard]] Matrix3 inverse() const
    {
        // TODO
        return Matrix3 {};
    }

    [[nodiscard]] Vector3<T> c1() const
    {
        return {r1.x, r2.x, r3.x};
    }
    [[nodiscard]] Vector3<T> c2() const
    {
        return {r1.y, r2.y, r3.y};
    }
    [[nodiscard]] Vector3<T> c3() const
    {
        return {r1.z, r2.z, r3.z};
    }

    void clear()
    {
        r1.clear();
        r2.clear();
        r3.clear();
    }
};

template<typename T>
Matrix3<T> operator+(const Matrix3<T>& m1, const Matrix3<T>& m2)
{
    return {m1.r1 + m2.r1, m1.r2 + m2.r2, m1.r3 + m2.r3};
}

template<typename T>
Matrix3<T> operator-(const Matrix3<T>& m1, const Matrix3<T>& m2)
{
    return {m1.r1 - m2.r1, m1.r2 - m2.r2, m1.r3 - m2.r3};
}

template<typename T>
Matrix3<T> operator*(const Matrix3<T>& m1, const Matrix3<T>& m2)
{
    Matrix3<T> result;
    result.r1.x = m1.r1.dot(m2.c1());
    result.r1.y = m1.r1.dot(m2.c2());
    result.r1.z = m1.r1.dot(m2.c3());
    result.r2.x = m1.r2.dot(m2.c1());
    result.r2.y = m1.r2.dot(m2.c2());
    result.r2.z = m1.r2.dot(m2.c3());
    result.r3.x = m1.r3.dot(m2.c1());
    result.r3.y = m1.r3.dot(m2.c2());
    result.r3.z = m1.r3.dot(m2.c3());
    return result;
}

template<typename T>
Vector3<T> operator*(const Matrix3<T>& m, const Vector3<T>& v)
{
    Vector3<T> result;
    result.x = m.r1.dot(v);
    result.y = m.r2.dot(v);
    result.z = m.r3.dot(v);
    return result;
}

template<typename T>
Matrix3<T> operator*(const Matrix3<T>& m, const T c)
{
    return {m.r1 * c, m.r2 * c, m.r3 * c};
}

template<typename T>
Matrix3<T> operator*(const T c, const Matrix3<T>& m)
{
    return m * c;
}

template<typename T>
Matrix3<T> operator/(const Matrix3<T>& m, const T c)
{
    return {m.r1 / c, m.r2 / c, m.r3 / c};
}

} // namespace Lionheart