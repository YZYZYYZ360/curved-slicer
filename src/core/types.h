#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace cslc {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3() = default;
    constexpr Vec3(double x_value, double y_value, double z_value)
        : x(x_value), y(y_value), z(z_value) {}

    double operator[](std::size_t axis) const
    {
        return axis == 0 ? x : (axis == 1 ? y : z);
    }

    double& operator[](std::size_t axis)
    {
        return axis == 0 ? x : (axis == 1 ? y : z);
    }
};

inline Vec3 operator+(const Vec3& lhs, const Vec3& rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

inline Vec3 operator-(const Vec3& lhs, const Vec3& rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

inline Vec3 operator*(const Vec3& value, double scale)
{
    return {value.x * scale, value.y * scale, value.z * scale};
}

inline Vec3 operator*(double scale, const Vec3& value)
{
    return value * scale;
}

inline Vec3 operator/(const Vec3& value, double scale)
{
    return {value.x / scale, value.y / scale, value.z / scale};
}

inline double dot(const Vec3& lhs, const Vec3& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline Vec3 cross(const Vec3& lhs, const Vec3& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

inline double squaredNorm(const Vec3& value)
{
    return dot(value, value);
}

inline double norm(const Vec3& value)
{
    return std::sqrt(squaredNorm(value));
}

inline Vec3 normalized(const Vec3& value)
{
    const double length = norm(value);
    return length > 0.0 ? value / length : Vec3{};
}

struct Tri {
    std::size_t v0 = 0;
    std::size_t v1 = 0;
    std::size_t v2 = 0;
};

struct AABB {
    Vec3 min{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
    };
    Vec3 max{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    };

    bool valid() const
    {
        return min.x <= max.x && min.y <= max.y && min.z <= max.z;
    }

    void expand(const Vec3& point)
    {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
    }

    Vec3 size() const
    {
        return valid() ? Vec3{max.x - min.x, max.y - min.y, max.z - min.z} : Vec3{};
    }

    AABB padded(double padding) const
    {
        if (!valid()) {
            return {};
        }

        return {
            {min.x - padding, min.y - padding, min.z - padding},
            {max.x + padding, max.y + padding, max.z + padding},
        };
    }
};

}  // namespace cslc

