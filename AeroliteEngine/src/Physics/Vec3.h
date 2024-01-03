#ifndef VEC3_H
#define VEC3_H

#include <stdexcept>
#include <cassert>
#include "Precision.h"

namespace Aerolite {

    // The Vec3 class represents a 3-dimensional vector.
    // It's designed for operations common in 3D physics and graphics, such as addition,
    // subtraction, scaling, and cross product. It also includes methods for calculating
    // magnitude, normalization, and dot product.
    
    struct Vec3 {
    public:
        real x;  // The x-coordinate of the vector
        real y;  // The y-coordinate of the vector
        real z;  // The z-coordinate of the vector

        // Default constructor. Initializes a new Vec3 with default values.
        Vec3() noexcept = default;

        // Parameterized constructor. Initializes a new Vec3 with specified x, y, and z values.
        Vec3(real x, real y, real z) noexcept;

        // Default destructor. (Trivial in this case as no dynamic memory allocation is involved)
        ~Vec3() noexcept = default;

        // Adds a Vec3 to this vector. (Inline for performance)
        void Add(const Vec3& v) noexcept;

        // Subtracts a Vec3 from this vector. (Inline for performance)
        void Sub(const Vec3& v) noexcept;

        // Scales this vector by a scalar value. (Inline for performance)
        void Scale(const real n) noexcept;

        // Returns the magnitude (length) of the vector.
        real Magnitude() const noexcept;

        // Returns the squared magnitude of the vector, which is more efficient
        // than Magnitude() as it avoids the square root operation.
        constexpr real MagnitudeSquared() const noexcept
        {
            return x * x + y * y + z * z;
        };

        // Normalizes the vector (makes its length 1) and returns a reference to it.
        Vec3& Normalize() noexcept;

        // Returns a new Vec3 representing the unit vector (vector of length 1) of this vector.
        Vec3 UnitVector() const noexcept;

        // Computes the dot product of this vector with another Vec3.
        constexpr real Dot(const Vec3& v) const noexcept
        {
            return x * v.x + y * v.y + z * v.z;
        };

        // Computes the cross product of this vector with another Vec3 and returns the resulting Vec3.
        Vec3 Cross(const Vec3& v) const noexcept;

         // Computes an orthonormal basis out of two vectors a & b. Does not assume a & b are already orthogonal.
        void MakeOrthonormalBasis(Vec3* a, Vec3* b, Vec3* c);

        // Assignment operator. Assigns the values of the given Vec3 to this Vec3.
        Vec3& operator=(const Vec3& v) noexcept;

        // Equality comparison operator. Returns true if this Vec3 is equal to the given Vec3.
        constexpr bool operator==(const Vec3& v) const noexcept
        {
            return x == v.x && y == v.y && z == v.z;
        };

        // Inequality comparison operator. Returns true if this Vec3 is not equal to the given Vec3.
        constexpr bool operator!=(const Vec3& v) const noexcept
        {
            return !(*this == v);
        };

        // Addition operator. Returns a new Vec3 that is the sum of this Vec3 and the given Vec3.
        Vec3 operator+(const Vec3& v) const noexcept;

        // Subtraction operator. Returns a new Vec3 that is the result of subtracting the given Vec3 from this Vec3.
        Vec3 operator-(const Vec3& v) const noexcept;

        // Scalar multiplication operator. Returns a new Vec3 that is this Vec3 scaled by the given scalar value.
        Vec3 operator*(const real n) const noexcept;

        // Scalar division operator. Returns a new Vec3 that is this Vec3 divided by the given scalar value.
        // Asserts in debug mode if the scalar value is zero.
        Vec3 operator/(const real n) const;

        // Unary minus operator. Returns a new Vec3 that is the negation of this Vec3.
        Vec3 operator-() const noexcept;

        // Addition assignment operator. Adds the given Vec3 to this Vec3 and returns a reference to this Vec3.
        Vec3& operator+=(const Vec3& v) noexcept;

        // Subtraction assignment operator. Subtracts the given Vec3 from this Vec3 and returns a reference to this Vec3.
        Vec3& operator-=(const Vec3& v) noexcept;

        // Scalar multiplication assignment operator. Scales this Vec3 by the given scalar value and returns a reference to this Vec3.
        Vec3& operator*=(const real n) noexcept;

        // Scalar division assignment operator. Divides this Vec3 by the given scalar value and returns a reference to this Vec3.
        // Asserts in debug mode if the scalar value is zero.
        Vec3& operator/=(const real n);
    };
}

#endif // VEC3_H
