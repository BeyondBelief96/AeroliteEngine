#ifndef VECN_H
#define VECN_H

#include <array>
#include <stdexcept>
#include <cmath>

namespace Aerolite {

    template<std::size_t N>
    class VecN {
    private:
        std::array<real, N> components; // Array to store vector components

    public:
        // Default constructor: Initializes all components to zero.
        VecN() {
            components.fill(0.0);
        }

        void Zero(void) {
            for (int i = 0; i < N; i++)
            {
                components[i] = 0;
            }
        }

        // Access element (const and non-const versions).
        real operator[](std::size_t index) const {
            if (index >= N) {
                throw std::out_of_range("Index out of range");
            }
            return components[index];
        }

        real& operator[](std::size_t index) {
            if (index >= N) {
                throw std::out_of_range("Index out of range");
            }
            return components[index];
        }

        // Calculate and return the dot product of this vector with another vector.
        real Dot(const VecN& other) const {
            real sum = 0.0;
            for (std::size_t i = 0; i < N; ++i) {
                sum += components[i] * other.components[i];
            }
            return sum;
        }

        // Calculate and return the Euclidean norm (length) of the vector.
        real Norm() const {
            return std::sqrt(this->Dot(*this));
        }

        // Normalize the vector (make it unit length).
        void Normalize() {
            real norm = this->Norm();
            if (norm == 0) {
                throw std::runtime_error("Cannot normalize a zero vector.");
            }
            for (auto& component : components) {
                component /= norm;
            }
        }

        // Addition of two vectors.
        VecN operator+(const VecN& other) const {
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = components[i] + other.components[i];
            }
            return result;
        }

        // Subtraction of two vectors.
        VecN operator-(const VecN& other) const {
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = components[i] - other.components[i];
            }
            return result;
        }

        // Scalar multiplication operator.
        VecN operator*(real scalar) const {
            VecN result;
            for (std::size_t i = 0; i < N; ++i) {
                result.components[i] = components[i] * scalar;
            }
            return result;
        }

        // In-place addition.
        VecN& operator+=(const VecN& other) {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] += other.components[i];
            }
            return *this;
        }

        // In-place subtraction.
        VecN& operator-=(const VecN& other) {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] -= other.components[i];
            }
            return *this;
        }

        // In-place scalar multiplication.
        VecN& operator*=(real scalar) {
            for (std::size_t i = 0; i < N; ++i) {
                components[i] *= scalar;
            }
            return *this;
        }

        // Additional methods and operator overloads can be added as needed.
    };
}

#endif
