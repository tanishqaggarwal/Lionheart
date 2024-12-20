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

    [[nodiscard]] T dot(const Vector3<T>& other) const noexcept
    {
        return this->x * other.x + this->y * other.y + this->z * other.z;
    }

    [[nodiscard]] Vector3<T> cross(const Vector3<T>& other) const noexcept
    {
        Vector3 result;
        result.x = this->y * other.z - other.y * this->z;
        result.y = this->z * other.x - other.z * this->x;
        result.z = this->x * other.y - other.x * this->y;
        return result;
    }

    [[nodiscard]] T norm() const noexcept
    {
        return std::sqrt(x*x + y*y + z*z);
    }

    Vector3 operator+=(const Vector3& other) noexcept
    {
        *this = *this + other;
        return *this;
    }

    Vector3 operator-=(const Vector3& other) noexcept
    {
        *this = *this - other;
        return *this;
    }

    void clear() noexcept
    {
        x = T{0.0};
        y = T{0.0};
        z = T{0.0};
    }

    [[nodiscard]] T& operator()(size_t i)
    {
        switch(i)
        {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::runtime_error{"invalid vector access"};
        }
    }

    [[nodiscard]] const T& operator()(size_t i) const
    {
        switch(i)
        {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::runtime_error{"invalid vector access"};
        }
    }
};

template<typename T>
Vector3<T> operator+(const Vector3<T>& v1, const Vector3<T>& v2) noexcept
{
    return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

template<typename T>
Vector3<T> operator-(const Vector3<T>& v1, const Vector3<T>& v2) noexcept
{
    return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}

template<typename T>
Vector3<T> operator*(const Vector3<T>& v, const T c) noexcept
{
    return {v.x * c, v.y * c, v.z * c};
}

template<typename T>
Vector3<T> operator*(const T c, const Vector3<T>& v) noexcept
{
    return v * c;
}

template<typename T>
Vector3<T> operator/(const Vector3<T>& v, const T c) noexcept
{
    return {v.x / c, v.y / c, v.z / c};
}

template<typename T>
struct Matrix3
{
    Vector3<T> r1;
    Vector3<T> r2;
    Vector3<T> r3;

    Matrix3 operator+=(const Matrix3& other) noexcept
    {
        *this = *this + other;
        return *this;
    }

    Matrix3 operator-=(const Matrix3& other) noexcept
    {
        *this = *this - other;
        return *this;
    }

    [[nodiscard]] T determinant() const noexcept
    {
        const auto& m = *this;
        return m.r1.x * (m.r2.y * m.r3.z - m.r3.y * m.r2.z) -
             m.r1.y * (m.r2.x * m.r3.z - m.r2.z * m.r3.x) +
             m.r1.z * (m.r2.x * m.r3.y - m.r2.y * m.r3.x);
    }

    [[nodiscard]] Matrix3 inverse() const noexcept
    {
        double invdet = 1 / determinant();

        const auto& m = *this;
        Matrix3 minv;
        minv.r1.x = (m.r2.y * m.r3.z - m.r3.y * m.r2.z);
        minv.r1.y = (m.r1.z * m.r3.y - m.r1.y * m.r3.z);
        minv.r1.z = (m.r1.y * m.r2.z - m.r1.z * m.r2.y);
        minv.r2.x = (m.r2.z * m.r3.x - m.r2.x * m.r3.z);
        minv.r2.y = (m.r1.x * m.r3.z - m.r1.z * m.r3.x);
        minv.r2.z = (m.r2.x * m.r1.z - m.r1.x * m.r2.z);
        minv.r3.x = (m.r2.x * m.r3.y - m.r3.x * m.r2.y);
        minv.r3.y = (m.r3.x * m.r1.y - m.r1.x * m.r3.y);
        minv.r3.z = (m.r1.x * m.r2.y - m.r2.x * m.r1.y);

        return minv * invdet;
    }

    [[nodiscard]] Vector3<T> c1() const noexcept
    {
        return {r1.x, r2.x, r3.x};
    }
    [[nodiscard]] Vector3<T> c2() const noexcept
    {
        return {r1.y, r2.y, r3.y};
    }
    [[nodiscard]] Vector3<T> c3() const noexcept
    {
        return {r1.z, r2.z, r3.z};
    }

    void clear() noexcept
    {
        r1.clear();
        r2.clear();
        r3.clear();
    }

    [[nodiscard]] T& operator()(size_t i, size_t j)
    {
        switch(i)
        {
            case 0: return r1(j);
            case 1: return r2(j);
            case 2: return r3(j);
            default: throw std::runtime_error{"invalid matrix access"};
        }
    }

    [[nodiscard]] const T& operator()(size_t i, size_t j) const
    {
        switch(i)
        {
            case 0: return r1(j);
            case 1: return r2(j);
            case 2: return r3(j);
            default: throw std::runtime_error{"invalid matrix access"};
        }
    }
};

template<typename T>
Matrix3<T> operator+(const Matrix3<T>& m1, const Matrix3<T>& m2) noexcept
{
    return {m1.r1 + m2.r1, m1.r2 + m2.r2, m1.r3 + m2.r3};
}

template<typename T>
Matrix3<T> operator-(const Matrix3<T>& m1, const Matrix3<T>& m2) noexcept
{
    return {m1.r1 - m2.r1, m1.r2 - m2.r2, m1.r3 - m2.r3};
}

template<typename T>
Matrix3<T> operator*(const Matrix3<T>& m1, const Matrix3<T>& m2) noexcept
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
Vector3<T> operator*(const Matrix3<T>& m, const Vector3<T>& v) noexcept
{
    Vector3<T> result;
    result.x = m.r1.dot(v);
    result.y = m.r2.dot(v);
    result.z = m.r3.dot(v);
    return result;
}

template<typename T>
Matrix3<T> operator*(const Matrix3<T>& m, const T c) noexcept
{
    return {m.r1 * c, m.r2 * c, m.r3 * c};
}

template<typename T>
Matrix3<T> operator*(const T c, const Matrix3<T>& m) noexcept
{
    return m * c;
}

template<typename T>
Matrix3<T> operator/(const Matrix3<T>& m, const T c) noexcept
{
    return {m.r1 / c, m.r2 / c, m.r3 / c};
}

} // namespace Lionheart