#ifndef VEC2_H
#define VEC2_H

#include <cmath>
#include <stdexcept>

namespace Aerolite {
    // The Vec2 struct represents a 2-dimensional vector.
    // It's designed for operations common in 2D physics and graphics, such as addition,
    // subtraction, scaling, and rotation. It also includes methods for calculating
    // magnitude, normalization, and vector products.
    
    struct Vec2 {
        float x = 0.0f;  // The x-coordinate of the vector
        float y = 0.0f;  // The y-coordinate of the vector

        // Default constructor. Initializes a new Vec2 with default values.
         Vec2() noexcept = default;

        // Parameterized constructor. Initializes a new Vec2 with specified x and y values.
         Vec2(float x, float y) noexcept;

        // Default destructor. (Trivial in this case as no dynamic memory allocation is involved)
         ~Vec2() noexcept = default;

        // Adds a Vec2 to this vector.
        constexpr inline void Add(const Vec2& v) noexcept
        {
            x += v.x;
            y += v.y;
        }

        // Subtracts a Vec2 from this vector.
        constexpr inline void Sub(const Vec2& v) noexcept
        {
            x -= v.x;
            y -= v.y;
        }

        // Scales this vector by a scalar value.
        constexpr inline void Scale(const float n) noexcept
        {
            x *= n;
            y *= n;
        }

        // Rotates the vector by a specified angle (in radians) and returns the result.
        // The original vector remains unchanged.
         Vec2 Rotate(const float angle) const noexcept;

        // Returns the magnitude (length) of the vector.
        float Magnitude() const noexcept;

        // Returns the squared magnitude of the vector, which is more efficient
        // than Magnitude() as it avoids the square root operation.
        constexpr float MagnitudeSquared(void) const noexcept
        {
            return x * x + y * y;
        }

        // Normalizes the vector (makes its length 1);
        void Normalize() noexcept;

        // Returns a new Vec2 representing the unit vector (vector of length 1) of this vector.
        Vec2 UnitVector() const noexcept;

        // Returns a new Vec2 that is perpendicular (normal) to this vector.
        Vec2 Normal() const noexcept;

        // Computes the dot product of this vector with another Vec2.
        constexpr inline float Dot(const Vec2& v) const noexcept
        {
            return x * v.x + y * v.y;
        };

        // Computes the cross product (scalar value) of this vector with another Vec2.
        // Note: In 2D, the cross product is not a vector but a scalar indicating
        // the magnitude of the vector perpendicular to the plane formed by the two vectors.
        constexpr inline float Cross(const Vec2& v) const noexcept
        {
            return (x * v.y) - (y * v.x);
        };

        // Assignment operator. Assigns the values of the given Vec2 to this Vec2.
        Vec2& operator=(const Vec2& v) noexcept;

        // Equality comparison operator. Returns true if this Vec2 is equal to the given Vec2.
        bool operator==(const Vec2& v) const noexcept;

        // Inequality comparison operator. Returns true if this Vec2 is not equal to the given Vec2.
        bool operator!=(const Vec2& v) const noexcept;

        // Addition operator. Returns a new Vec2 that is the sum of this Vec2 and the given Vec2.
        Vec2 operator+(const Vec2& v) const noexcept;

        // Subtraction operator. Returns a new Vec2 that is the result of subtracting the given Vec2 from this Vec2.
        Vec2 operator-(const Vec2& v) const noexcept;

        // Scalar multiplication operator. Returns a new Vec2 that is this Vec2 scaled by the given scalar value.
        Vec2 operator*(const float n) const noexcept;

        // Scalar division operator. Returns a new Vec2 that is this Vec2 divided by the given scalar value.
        // Throws a runtime error if the scalar value is zero.
        Vec2 operator/(const float n) const;

        // Unary minus operator. Returns a new Vec2 that is the negation of this Vec2.
        Vec2 operator-() const noexcept;

        // Addition assignment operator. Adds the given Vec2 to this Vec2 and returns a reference to this Vec2.
        Vec2& operator+=(const Vec2& v) noexcept;

        // Subtraction assignment operator. Subtracts the given Vec2 from this Vec2 and returns a reference to this Vec2.
        Vec2& operator-=(const Vec2& v) noexcept;

        // Scalar multiplication assignment operator. Scales this Vec2 by the given scalar value and returns a reference to this Vec2.
        Vec2& operator*=(const float n) noexcept;

        // Scalar division assignment operator. Divides this Vec2 by the given scalar value and returns a reference to this Vec2.
        // Throws a runtime error if the scalar value is zero.
        Vec2& operator/=(const float n);
    };
}

#endif // VEC2_H
