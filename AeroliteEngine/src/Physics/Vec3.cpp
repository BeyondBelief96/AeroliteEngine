#include "Vec3.h"
#include <cmath> // for std::hypot
#include <stdexcept>
#include <cassert>

namespace Aerolite {
    // Default member initializers
    Vec3::Vec3(float x, float y, float z) noexcept : x(x), y(y), z(z) {}

    // Inline simple methods for performance
    inline void Vec3::Add(const Vec3& v) noexcept {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    inline void Vec3::Sub(const Vec3& v) noexcept {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    inline void Vec3::Scale(const float n) noexcept {
        x *= n;
        y *= n;
        z *= n;
    }

    // Use std::hypot for better numerical stability
    inline float Vec3::Magnitude() const noexcept {
        return sqrtf(x * x + y * y + z * z);
    }

    // constexpr for compile-time calculations
    constexpr float Vec3::MagnitudeSquared() const noexcept {
        return x * x + y * y + z * z;
    }

    Vec3& Vec3::Normalize() noexcept {
        float mag = Magnitude();
        if (mag > 0) {
            x /= mag;
            y /= mag;
            z /= mag;
        }
        return *this;
    }

    Vec3 Vec3::UnitVector() const noexcept {
        Vec3 unit = *this;
        return unit.Normalize();
    }

    void Vec3::MakeOrthonormalBasis(Vec3* a, Vec3* b, Vec3* c)
    {
        *a = a->Normalize();
        (*c) = a->Cross(*b);
        if (c->MagnitudeSquared() == 0.0) return;
        *c = c->Normalize();
        *b = c->Cross(*a);
    }

    constexpr float Vec3::Dot(const Vec3& v) const noexcept {
        return x * v.x + y * v.y + z * v.z;
    }

    Vec3 Vec3::Cross(const Vec3& v) const noexcept {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    Vec3& Vec3::operator=(const Vec3& v) noexcept {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }

    constexpr bool Vec3::operator==(const Vec3& v) const noexcept {
        return x == v.x && y == v.y && z == v.z;
    }

    constexpr bool Vec3::operator!=(const Vec3& v) const noexcept {
        return !(*this == v);
    }

    Vec3 Vec3::operator+(const Vec3& v) const noexcept {
        return Vec3(x + v.x, y + v.y, z + v.z);
    }

    Vec3 Vec3::operator-(const Vec3& v) const noexcept {
        return Vec3(x - v.x, y - v.y, z - v.z);
    }

    Vec3 Vec3::operator*(const float n) const noexcept {
        return Vec3(x * n, y * n, z * n);
    }

    Vec3 Vec3::operator/(const float n) const {
        assert(n != 0); // Assert in debug mode
        if (n == 0) {
            throw std::runtime_error("Division by zero error in Vec3::operator/.");
        }
        return Vec3(x / n, y / n, z / n);
    }

    Vec3 Vec3::operator-() const noexcept {
        return Vec3(-x, -y, -z);
    }

    Vec3& Vec3::operator+=(const Vec3& v) noexcept {
        Add(v);
        return *this;
    }

    Vec3& Vec3::operator-=(const Vec3& v) noexcept {
        Sub(v);
        return *this;
    }

    Vec3& Vec3::operator*=(const float n) noexcept {
        Scale(n);
        return *this;
    }

    Vec3& Vec3::operator/=(const float n) {
        assert(n != 0); // Assert in debug mode
        if (n == 0) {
            throw std::runtime_error("Division by zero error in Vec3::operator/=.");
        }
        x /= n;
        y /= n;
        z /= n;
        return *this;
    }
}
