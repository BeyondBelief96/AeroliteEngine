#include "AeroVec3.h"
#include <stdexcept>
#include <cassert>

namespace Aerolite {
    // Default member initializers
    AeroVec3::AeroVec3(const real x, const real y, const real z) noexcept : x(x), y(y), z(z) {}

    // Inline simple methods for performance
    inline void AeroVec3::Add(const AeroVec3& v) noexcept {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    inline void AeroVec3::Sub(const AeroVec3& v) noexcept {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    inline void AeroVec3::Scale(const real n) noexcept {
        x *= n;
        y *= n;
        z *= n;
    }

    // Use std::hypot for better numerical stability
    inline real AeroVec3::Magnitude() const noexcept {
        return sqrtf(x * x + y * y + z * z);
    }

    AeroVec3& AeroVec3::Normalize() noexcept {
	    if (const real mag = Magnitude(); mag > 0) {
            x /= mag;
            y /= mag;
            z /= mag;
        }
        return *this;
    }

    AeroVec3 AeroVec3::UnitVector() const noexcept {
        AeroVec3 unit;
        unit = *this;
        return unit.Normalize();
    }

    void AeroVec3::MakeOrthonormalBasis(AeroVec3& a, AeroVec3& b, AeroVec3& c)
    {
        a = a.Normalize();
        c = a.Cross(b);
        if (c.MagnitudeSquared() == 0.0) return;
        c = c.Normalize();
        b = c.Cross(a);
    }

    AeroVec3 AeroVec3::Cross(const AeroVec3& v) const noexcept {
        return AeroVec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    AeroVec3 AeroVec3::operator+(const AeroVec3& v) const noexcept {
        return AeroVec3(x + v.x, y + v.y, z + v.z);
    }

    AeroVec3 AeroVec3::operator-(const AeroVec3& v) const noexcept {
        return AeroVec3(x - v.x, y - v.y, z - v.z);
    }

    AeroVec3 AeroVec3::operator*(const real n) const noexcept {
        return AeroVec3(x * n, y * n, z * n);
    }

    AeroVec3 AeroVec3::operator/(const real n) const {
        assert(n != 0); // Assert in debug mode
        if (n == 0) {
            throw std::runtime_error("Division by zero error in AeroVec3::operator/.");
        }
        return AeroVec3(x / n, y / n, z / n);
    }

    AeroVec3 AeroVec3::operator-() const noexcept {
        return AeroVec3(-x, -y, -z);
    }

    AeroVec3& AeroVec3::operator+=(const AeroVec3& v) noexcept {
        Add(v);
        return *this;
    }

    AeroVec3& AeroVec3::operator-=(const AeroVec3& v) noexcept {
        Sub(v);
        return *this;
    }

    AeroVec3& AeroVec3::operator*=(const real n) noexcept {
        Scale(n);
        return *this;
    }

    AeroVec3& AeroVec3::operator/=(const real n) {
        assert(n != 0); // Assert in debug mode
        if (n == 0) {
            throw std::runtime_error("Division by zero error in AeroVec3::operator/=.");
        }
        x /= n;
        y /= n;
        z /= n;
        return *this;
    }
}
