#include <cmath>
#include <stdexcept>
#include "Vec2.h"
#include "Precision.h"

namespace Aerolite {
    // Constructor: Initializes the vector with specified x and y values.
    Vec2::Vec2(Aerolite::real x, Aerolite::real y) noexcept : x(x), y(y) {}

    // Rotate: Rotates the vector by a given angle in radians.
    Vec2 Vec2::Rotate(const Aerolite::real angle) const noexcept {
        // Uses standard 2D rotation formula.
        if (std::is_same<Aerolite::real, float>::value)
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
    Aerolite::real Vec2::Magnitude() const noexcept {
        // std::hypot is used for numerical stability.
        return std::hypot(x, y);
    }

    // Normalize: Scales the vector to a unit length.
    void Vec2::Normalize() noexcept {
        Aerolite::real length = Magnitude();
        if (length != 0.0f) {
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

    Aerolite::real Vec2::DistanceTo(Vec2 v) const noexcept {
        // Calculates the distance between this point and another point represented by vector v.
        // The distance is computed as the square root of the sum of the squares of the differences 
        // in the corresponding components.
        // The std::sqrtf or std::sqrt function is used based on whether Aerolite::real is float or double.
        Aerolite::real dx = v.x - x;
        Aerolite::real dy = v.y - y;
        if (std::is_same<Aerolite::real, float>::value) {
            return std::sqrtf(dx * dx + dy * dy);
        }
        else if (std::is_same<Aerolite::real, double>::value) {
            return std::sqrt(dx * dx + dy * dy);
        }
        else {
            // This else case handles potential future changes in the type of Aerolite::real.
            // A static assertion or alternative handling can be added here as necessary.
            // For now, we default to using double precision.
            return std::sqrt(dx * dx + dy * dy);
        }
    }

    // Assignment operator.
    Vec2& Vec2::operator=(const Vec2& v) noexcept {
        x = v.x;
        y = v.y;
        return *this;
    }

    // Equality comparison operator.
    bool Vec2::operator==(const Vec2& v) const noexcept {
        // Precision comparison to handle floating-point inaccuracies.
        return Aerolite::AreEqual(x, v.x, Aerolite::epsilon) &&
            Aerolite::AreEqual(y, v.y, Aerolite::epsilon);
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
    Vec2 Vec2::operator*(const Aerolite::real n) const noexcept {
        return Vec2(x * n, y * n);
    }

    // Scalar division operator.
    Vec2 Vec2::operator/(const Aerolite::real n) const {
        if (n == 0) throw std::runtime_error("Division by zero error in Vec2 operator /");
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
    Vec2& Vec2::operator*=(const Aerolite::real n) noexcept {
        Scale(n);
        return *this;
    }

    // Scalar division assignment operator.
    Vec2& Vec2::operator/=(const Aerolite::real n) {
        if (n == 0) throw std::runtime_error("Division by zero error in Vec2 operator /=");
        x /= n;
        y /= n;
        return *this;
    }
}
