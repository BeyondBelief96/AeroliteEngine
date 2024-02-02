#ifndef VEC3_H
#define VEC3_H

#include "Precision.h"

namespace Aerolite {

    // The AeroVec3 class represents a 3-dimensional vector.
    // It's designed for operations common in 3D physics and graphics, such as addition,
    // subtraction, scaling, and cross product. It also includes methods for calculating
    // magnitude, normalization, and dot product.
    
    struct AeroVec3 {
    public:
        real x;  // The x-coordinate of the vector
        real y;  // The y-coordinate of the vector
        real z;  // The z-coordinate of the vector

        // Default constructor. Initializes a new AeroVec3 with default values.
        AeroVec3() noexcept = default;

        // Parameterized constructor. Initializes a new AeroVec3 with specified x, y, and z values.
        AeroVec3(real x, real y, real z) noexcept;

        // Copy constructor.
        AeroVec3(AeroVec3& v) noexcept = default;

        // Const version copy constructor
        AeroVec3(const AeroVec3& v) noexcept = default;

        // Move constructor.
        AeroVec3(AeroVec3&& v) noexcept = default;

        // Default destructor. (Trivial in this case as no dynamic memory allocation is involved)
        ~AeroVec3() noexcept = default;

        // Adds a AeroVec3 to this vector. (Inline for performance)
        void Add(const AeroVec3& v) noexcept;

        // Subtracts a AeroVec3 from this vector. (Inline for performance)
        void Sub(const AeroVec3& v) noexcept;

        // Scales this vector by a scalar value. (Inline for performance)
        void Scale(const real n) noexcept;

        // Returns the magnitude (length) of the vector.
        [[nodiscard]] real Magnitude() const noexcept;

        // Returns the squared magnitude of the vector, which is more efficient
        // than Magnitude() as it avoids the square root operation.
        [[nodiscard]] constexpr real MagnitudeSquared() const noexcept
        {
            return x * x + y * y + z * z;
        }

        // Normalizes the vector (makes its length 1) and returns a reference to it.
        AeroVec3& Normalize() noexcept;

        // Returns a new AeroVec3 representing the unit vector (vector of length 1) of this vector.
        [[nodiscard]] AeroVec3 UnitVector() const noexcept;

        // Computes the dot product of this vector with another AeroVec3.
        [[nodiscard]] constexpr real Dot(const AeroVec3& v) const noexcept
        {
            return x * v.x + y * v.y + z * v.z;
        }

        // Computes the cross product of this vector with another AeroVec3 and returns the resulting AeroVec3.
        AeroVec3 Cross(const AeroVec3& v) const noexcept;

         // Computes an orthonormal basis out of two vectors a & b. Does not assume a & b are already orthogonal.
        static void MakeOrthonormalBasis(AeroVec3& a, AeroVec3& b, AeroVec3& c);

        // Assignment operator. Assigns the values of the given AeroVec3 to this AeroVec3.
        AeroVec3& operator=(const AeroVec3& v) noexcept = default;

        // Move assignment operator. Uses move semantic to assign a AeroVec3's values to this AeroVec3.
        AeroVec3& operator=(AeroVec3&& v) noexcept = default;

        // Equality comparison operator. Returns true if this AeroVec3 is equal to the given AeroVec3.
    	bool operator==(const AeroVec3& v) const noexcept
        {
            return AreEqual(x, v.x, EPSILON) && AreEqual(y, v.y, EPSILON) && AreEqual(z, v.z, EPSILON);
        }

        // Inequality comparison operator. Returns true if this AeroVec3 is not equal to the given AeroVec3.
    	bool operator!=(const AeroVec3& v) const noexcept
        {
            return !(*this == v);
        }

        // Addition operator. Returns a new AeroVec3 that is the sum of this AeroVec3 and the given AeroVec3.
        AeroVec3 operator+(const AeroVec3& v) const noexcept;

        // Subtraction operator. Returns a new AeroVec3 that is the result of subtracting the given AeroVec3 from this AeroVec3.
        AeroVec3 operator-(const AeroVec3& v) const noexcept;

        // Scalar multiplication operator. Returns a new AeroVec3 that is this AeroVec3 scaled by the given scalar value.
        AeroVec3 operator*(const real n) const noexcept;

        // Scalar division operator. Returns a new AeroVec3 that is this AeroVec3 divided by the given scalar value.
        // Asserts in debug mode if the scalar value is zero.
        AeroVec3 operator/(const real n) const;

        // Unary minus operator. Returns a new AeroVec3 that is the negation of this AeroVec3.
        AeroVec3 operator-() const noexcept;

        // Addition assignment operator. Adds the given AeroVec3 to this AeroVec3 and returns a reference to this AeroVec3.
        AeroVec3& operator+=(const AeroVec3& v) noexcept;

        // Subtraction assignment operator. Subtracts the given AeroVec3 from this AeroVec3 and returns a reference to this AeroVec3.
        AeroVec3& operator-=(const AeroVec3& v) noexcept;

        // Scalar multiplication assignment operator. Scales this AeroVec3 by the given scalar value and returns a reference to this AeroVec3.
        AeroVec3& operator*=(const real n) noexcept;

        // Scalar division assignment operator. Divides this AeroVec3 by the given scalar value and returns a reference to this AeroVec3.
        // Asserts in debug mode if the scalar value is zero.
        AeroVec3& operator/=(const real n);
    };
}

#endif // VEC3_H
