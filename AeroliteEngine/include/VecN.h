#ifndef VECN_H
#define VECN_H

#include <array>
#include <stdexcept>
#include <cmath>

namespace Aerolite {

    // Template class for an N-dimensional vector.
    template<std::size_t N>
    class VecN {
    private:
        std::array<real, N> components; // Stores the components of the vector.

    public:
        // Default constructor initializes all components to zero.
        VecN() {
            components.fill(0.0);  // Fill all elements with 0.0.
        }

        // Sets all components of the vector to zero.
        void Zero(void) {
            for (int i = 0; i < N; i++) {
                components[i] = 0;
            }
        }

        // Const and non-const versions of the operator[] for element access.
        real operator[](std::size_t index) const {
            if (index >= N) {
                throw std::out_of_range("Index out of range"); // Range check for safety.
            }
            return components[index];
        }

        real& operator[](std::size_t index) {
            if (index >= N) {
                throw std::out_of_range("Index out of range"); // Range check for safety.
            }
            return components[index];
        }

        // Calculates and returns the dot product with another VecN.
        real Dot(const VecN& other) const {
            real sum = 0.0;
            for (std::size_t i = 0; i < N; ++i) {
                sum += components[i] * other.components[i]; // Sum of products of corresponding components.
            }
            return sum;
        }

        // Calculates and returns the Euclidean norm (length) of the vector.
        real Norm() const {
            return std::sqrt(this->Dot(*this)); // Square root of dot product with itself.
        }

        // Normalizes the vector to make it unit length.
        void Normalize() {
            real norm = this->Norm();
            if (norm == 0) {
                throw std::runtime_error("Cannot normalize a zero vector."); // Error on zero-length vector.
            }
            for (auto& component : components) {
                component /= norm; // Scale each component by the norm.
            }
        }

        // Adds two vectors and returns the result.
        VecN operator+(const VecN& other) const {
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = components[i] + other.components[i]; // Component-wise addition.
            }
            return result;
        }

        // Subtracts one vector from another and returns the result.
        VecN operator-(const VecN& other) const {
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = components[i] - other.components[i]; // Component-wise subtraction.
            }
            return result;
        }

        // Multiplies the vector by a scalar and returns the result.
        VecN operator*(real scalar) const {
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = components[i] * scalar; // Scalar multiplication.
            }
            return result;
        }

        // In-place addition of another vector.
        VecN& operator+=(const VecN& other) {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] += other.components[i]; // Component-wise in-place addition.
            }
            return *this;
        }

        // In-place subtraction of another vector.
        VecN& operator-=(const VecN& other) {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] -= other.components[i]; // Component-wise in-place subtraction.
            }
            return *this;
        }

        // In-place scalar multiplication.
        VecN& operator*=(real scalar) {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] *= scalar; // Scalar multiplication for each component.
            }
            return *this;
        }

        // Additional methods and operator overloads can be added as needed.
    };
}

#endif // VECN_H
