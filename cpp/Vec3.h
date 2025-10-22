#pragma once

#include <array>
#include <cmath>

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    double Norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    double NormSquared() const {
        return x * x + y * y + z * z;
    }

    Vec3 Normalized() const {
        double n = Norm();
        if (n == 0.0) {
            return Vec3(0.0, 0.0, 0.0);
        }
        double inv = 1.0 / n;
        return Vec3(x * inv, y * inv, z * inv);
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vec3& operator/=(double scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    std::array<double, 3> ToArray() const {
        return {x, y, z};
    }
};

inline Vec3 operator+(Vec3 lhs, const Vec3& rhs) {
    lhs += rhs;
    return lhs;
}

inline Vec3 operator-(Vec3 lhs, const Vec3& rhs) {
    lhs -= rhs;
    return lhs;
}

inline Vec3 operator*(Vec3 lhs, double scalar) {
    lhs *= scalar;
    return lhs;
}

inline Vec3 operator*(double scalar, Vec3 rhs) {
    rhs *= scalar;
    return rhs;
}

inline Vec3 operator/(Vec3 lhs, double scalar) {
    lhs /= scalar;
    return lhs;
}

inline double Dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 Cross(const Vec3& a, const Vec3& b) {
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

inline Vec3 ClampAxis(const Vec3& value, const std::array<bool, 3>& mask) {
    return Vec3(
        mask[0] ? value.x : 0.0,
        mask[1] ? value.y : 0.0,
        mask[2] ? value.z : 0.0
    );
}

inline Vec3 ZeroVec3() {
    return Vec3(0.0, 0.0, 0.0);
}
