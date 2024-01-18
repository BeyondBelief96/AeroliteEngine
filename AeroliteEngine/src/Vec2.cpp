#include <cmath>
#include <stdexcept>
#include "Vec2.h"
#include "Precision.h"

namespace Aerolite {
    // Constructor: Initializes the vector with specified x and y values.
    Vec2::Vec2(const real x, const real y) noexcept : x(x), y(y) {}

    // Rotate: Rotates the vector by a given angle in radians.
    Vec2 Vec2::Rotate(const real angle) const noexcept {
        // Uses standard 2D rotation formula.
        if (std::is_same_v<real, float>)
        {
            return Vec2(
                x * std::cosf(angle) - y * std::sinf(angle), // New x coordinate.
                x * std::sinf(angle) + y * std::cosf(angle)  // New y coordinate.
            );
        }
        else
        {
            return Vec2(
                x * std::cos(angle) - y * std::sin(angle), // New x coordinate.
                x * std::sin(angle) + y * std::cos(angle)  // New y coordinate.
            );
        } 
    }

    // Magnitude: Returns the length of the vector.
    real Vec2::Magnitude() const noexcept {
        // std::hypot is used for numerical stability.
        return std::hypot(x, y);
    }

    // Normalize: Scales the vector to a unit length.
    void Vec2::Normalize() noexcept {
	    if (const real length = Magnitude(); length != 0.0f) {
            x /= length;
            y /= length;
        }
        // Consider adding an else clause to handle the case where length is zero.
    }

    // UnitVector: Returns a unit vector in the direction of this vector.
    Vec2 Vec2::UnitVector() const noexcept {
        Vec2 result = *this;
        result.Normalize();
        return result;
    }

    // Normal: Returns a vector perpendicular to this vector.
    Vec2 Vec2::Normal() const noexcept {
        // Direct calculation of the perpendicular vector.
        Vec2 result(y, -x);
        result.Normalize();
        return result;
    }

    real Vec2::DistanceTo(Vec2 v) const noexcept {
        // Calculates the distance between this point and another point represented by vector v.
        // The distance is computed as the square root of the sum of the squares of the differences 
        // in the corresponding components.
        // The std::sqrtf or std::sqrt function is used based on whether Aerolite::real is float or double.
        const real dx = v.x - x;
        const real dy = v.y - y;
        if (std::is_same_v<real, float>) {
            return std::sqrtf(dx * dx + dy * dy);
        }
        else if (std::is_same_v<real, double>) {
            return std::sqrt(dx * dx + dy * dy);
        }
        else {
            // This else case handles potential future changes in the type of Aerolite::real.
            // A static assertion or alternative handling can be added here as necessary.
            // For now, we default to using double precision.
            return std::sqrt(dx * dx + dy * dy);
        }
    }

    // Equality comparison operator.
    bool Vec2::operator==(const Vec2& v) const noexcept {
        // Precision comparison to handle floating-point inaccuracies.
        return AreEqual(x, v.x, epsilon) &&
	        AreEqual(y, v.y, epsilon);
    }

    // Inequality comparison operator.
    bool Vec2::operator!=(const Vec2& v) const noexcept {
        return !(*this == v);
    }

    // Addition operator.
    Vec2 Vec2::operator+(const Vec2& v) const noexcept {
        return Vec2(x + v.x, y + v.y);
    }

    // Subtraction operator.
    Vec2 Vec2::operator-(const Vec2& v) const noexcept {
        return Vec2(x - v.x, y - v.y);
    }

    // Scalar multiplication operator.
    Vec2 Vec2::operator*(const real n) const noexcept {
        return Vec2(x * n, y * n);
    }

    // Scalar division operator.
    Vec2 Vec2::operator/(const real n) const {
        if (Aerolite::AreEqual(n, 0.0, epsilon)) throw std::runtime_error("Division by zero error in Vec2 operator /");
        return Vec2(x / n, y / n);
    }

    // Unary negation operator.
    Vec2 Vec2::operator-() const noexcept {
        return Vec2(-x, -y);
    }

    // Addition assignment operator.
    Vec2& Vec2::operator+=(const Vec2& v) noexcept {
        Add(v);
        return *this;
    }

    // Subtraction assignment operator.
    Vec2& Vec2::operator-=(const Vec2& v) noexcept {
        Sub(v);
        return *this;
    }

    // Scalar multiplication assignment operator.
    Vec2& Vec2::operator*=(const real n) noexcept {
        Scale(n);
        return *this;
    }

    // Scalar division assignment operator.
    Vec2& Vec2::operator/=(const real n) {
        if (AreEqual(n, 0.0, epsilon)) throw std::runtime_error("Division by zero error in Vec2 operator /=");
        x /= n;
        y /= n;
        return *this;
    }
}
