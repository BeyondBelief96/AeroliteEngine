#ifndef VEC2_H
#define VEC2_H

#include "Precision.h"

namespace Aerolite {

    // Represents a 2-dimensional vector, primarily used in 2D physics and graphics.
    struct Vec2 {
	    real x = 0.0f;  // The x-coordinate of the vector.
	    real y = 0.0f;  // The y-coordinate of the vector.

        // Default constructor initializes vector to (0, 0).
        Vec2() noexcept = default;

        Vec2(const Vec2& v) = default;

        Vec2(Vec2&& v) noexcept : x(v.x), y(v.y){}

        // Parameterized constructor for initializing vector with specific x and y values.
        Vec2(real x, real y) noexcept;

        // Default destructor. No dynamic memory allocation, so trivial.
        ~Vec2() noexcept = default;

        // Adds another Vec2 to this vector (component-wise addition).
        constexpr void Add(const Vec2& v) noexcept
        {
            x = this->x + v.x;
            y = this->y + v.y;
        }

        // Subtracts another Vec2 from this vector (component-wise subtraction).
        constexpr void Sub(const Vec2& v) noexcept
        {
            x = this->x - v.x;
            y = this->y - v.y;
        }

        // Scales this vector by a scalar value.
        constexpr void Scale(const real n) noexcept
        {
            x = x * n;
            y = y * n;
        }

        // Rotates the vector by a specified angle in radians and returns the resulting vector.
        [[nodiscard]] Vec2 Rotate(real angle) const noexcept;

        // Returns the magnitude (length) of the vector.
        [[nodiscard]] real Magnitude() const noexcept;

        // Returns the squared magnitude of the vector (more efficient than Magnitude).
        constexpr real MagnitudeSquared() const noexcept
        {
            return x * x + y * y;
        }

        // Normalizes the vector (making its length 1).
        void Normalize() noexcept;

        // Returns a new Vec2 that is the normalized (unit length) version of this vector.
        [[nodiscard]] Vec2 UnitVector() const noexcept;

        // Returns a new Vec2 that is perpendicular (normal) to this vector.
        [[nodiscard]] Vec2 Normal() const noexcept;

        // Computes the dot product of this vector with another Vec2.
        constexpr real Dot(const Vec2& v) const noexcept
        {
            return x * v.x + y * v.y;
        }

        // Calculates and returns the cross product (scalar) with another vector.
        constexpr real Cross(const Vec2& v) const noexcept {
            // In 2D, the cross product results in a scalar rather than a vector.
            // It is calculated as the determinant of a matrix formed by the two vectors.
            // It represents the area of the parallelogram formed by the two vectors.
            return (x * v.y) - (y * v.x);
        }

        // Calculates the distance to another vector.
	    [[nodiscard]] real DistanceTo(Vec2 v) const noexcept;

        // Assignment operator to assign the values from another Vec2 to this one.
        Vec2& operator=(const Vec2& v) noexcept = default;

        Vec2& operator=(Vec2&& v) noexcept = default;

        // Equality comparison operator to check if two Vec2 instances are equal.
        bool operator==(const Vec2& v) const noexcept;

        // Inequality comparison operator to check if two Vec2 instances are not equal.
        bool operator!=(const Vec2& v) const noexcept;

        // Addition operator to add two Vec2 instances.
        Vec2 operator+(const Vec2& v) const noexcept;

        // Subtraction operator to subtract one Vec2 from another.
        Vec2 operator-(const Vec2& v) const noexcept;

        // Scalar multiplication operator to scale the vector by a real number.
        Vec2 operator*(real n) const noexcept;

        // Scalar division operator to divide the vector by a real number.
        Vec2 operator/(real n) const;

        // Unary minus operator to negate the vector.
        Vec2 operator-() const noexcept;

        // Addition assignment operator to add a Vec2 to this vector and update this vector.
        Vec2& operator+=(const Vec2& v) noexcept;

        // Subtraction assignment operator to subtract a Vec2 from this vector and update this vector.
        Vec2& operator-=(const Vec2& v) noexcept;

        // Scalar multiplication assignment operator to scale this vector by a real number and update this vector.
        Vec2& operator*=(real n) noexcept;

        // Scalar division assignment operator to divide this vector by a real number and update this vector.
        Vec2& operator/=(real n);
    };
}

#endif // VEC2_H
