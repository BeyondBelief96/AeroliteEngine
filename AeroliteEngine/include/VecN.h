#ifndef VECN_H
#define VECN_H

#include <stdexcept>
#include <cassert>
#include "Precision.h"

namespace Aerolite {

    // The VecN class template represents an N-dimensional vector.
    template <std::size_t N>
    struct VecN {
    public:
        // Components of the vector
        real components[N];

        // Default constructor. Initializes a new VecN with default values.
        VecN() noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] = real(0);
            }
        }

        // Parameterized constructor. Initializes a new VecN with specified values.
        explicit VecN(const real values[N]) noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] = values[i];
            }
        }

        explicit VecN(const VecN& v) noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] = v[i];
            }
        }

        // Default destructor. (Trivial in this case as no dynamic memory allocation is involved)
        ~VecN() noexcept = default;

        // Adds a VecN to this vector. (Inline for performance)
        inline void Add(const VecN& v) noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] += v.components[i];
            }
        }

        // Subtracts a VecN from this vector. (Inline for performance)
        inline void Sub(const VecN& v) noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] -= v.components[i];
            }
        }

        // Scales this vector by a scalar value. (Inline for performance)
        inline void Scale(const real n) noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] *= n;
            }
        }

        // Returns the magnitude (length) of the vector.
        real Magnitude() const noexcept {
            real sum = real(0);
            for (std::size_t i = 0; i < N; ++i) {
                sum += components[i] * components[i];
            }
            return std::sqrt(sum);
        }

        // Returns the squared magnitude of the vector, which is more efficient
        // than Magnitude() as it avoids the square root operation.
        constexpr real MagnitudeSquared() const noexcept {
            real sum = real(0);
            for (std::size_t i = 0; i < N; ++i) {
                sum += components[i] * components[i];
            }
            return sum;
        }

        // Normalizes the vector (makes its length 1) and returns a reference to it.
        VecN& Normalize() noexcept {
            const real length = Magnitude();
            assert(length != real(0));
            for (std::size_t i = 0; i < N; ++i) {
                components[i] /= length;
            }
            return *this;
        }

        // Returns a new VecN representing the unit vector (vector of length 1) of this vector.
        VecN UnitVector() const noexcept {
            VecN result = *this;
            result.Normalize();
            return result;
        }

        // Computes the dot product of this vector with another VecN.
        constexpr real Dot(const VecN& v) const noexcept {
            real dotProduct = real(0);
            for (std::size_t i = 0; i < N; ++i) {
                dotProduct += components[i] * v.components[i];
            }
            return dotProduct;
        }

        // Assignment operator. Assigns the values of the given VecN to this VecN.
        VecN& operator=(const VecN& v) noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] = v.components[i];
            }
            return *this;
        }

        // Equality comparison operator. Returns true if this VecN is equal to the given VecN.
        constexpr bool operator==(const VecN& v) const noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                if (components[i] != v.components[i]) {
                    return false;
                }
            }
            return true;
        }

        // Inequality comparison operator. Returns true if this VecN is not equal to the given VecN.
        constexpr bool operator!=(const VecN& v) const noexcept {
            return !(*this == v);
        }

        // Addition operator. Returns a new VecN that is the sum of this VecN and the given VecN.
        VecN operator+(const VecN& v) const noexcept {
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = components[i] + v.components[i];
            }
            return result;
        }

        // Subtraction operator. Returns a new VecN that is the result of subtracting the given VecN from this VecN.
        VecN operator-(const VecN& v) const noexcept {
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = components[i] - v.components[i];
            }
            return result;
        }

        // Scalar multiplication operator. Returns a new VecN that is this VecN scaled by the given scalar value.
        VecN operator*(const real n) const noexcept {
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = components[i] * n;
            }
            return result;
        }

        // Scalar division operator. Returns a new VecN that is this VecN divided by the given scalar value.
        // Asserts in debug mode if the scalar value is zero.
        VecN operator/(const real n) const {
            assert(n != real(0));
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = components[i] / n;
            }
            return result;
        }

        // Unary minus operator. Returns a new VecN that is the negation of this VecN.
        VecN operator-() const noexcept {
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = -components[i];
            }
            return result;
        }

        // Addition assignment operator. Adds the given VecN to this VecN and returns a reference to this VecN.
        VecN& operator+=(const VecN& v) noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] += v.components[i];
            }
            return *this;
        }

        // Subtraction assignment operator. Subtracts the given VecN from this VecN and returns a reference to this VecN.
        VecN& operator-=(const VecN& v) noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] -= v.components[i];
            }
            return *this;
        }

        // Scalar multiplication assignment operator. Scales this VecN by the given scalar value and returns a reference to this VecN.
        VecN& operator*=(const real n) noexcept {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] *= n;
            }
            return *this;
        }

        // Scalar division assignment operator. Divides this VecN by the given scalar value and returns a reference to this VecN.
        // Asserts in debug mode if the scalar value is zero.
        VecN& operator/=(const real n) {
            assert(n != real(0));
            for (std::size_t i = 0; i < N; ++i) {
                components[i] /= n;
            }
            return *this;
        }
    };
}

#endif // VECN_H
