#ifndef MATRIX_MXN_H
#define MATRIX_MXN_H

#include "VecN.h"
#include <vector>
#include <stdexcept>

namespace Aerolite {

    template<std::size_t M, std::size_t N>
    class MatrixMxN {
    private:
        std::vector<VecN<N>> rows; // Rows of the matrix

    public:
        // Default constructor
        MatrixMxN() : rows(M) {}

        // Constructor with initial values
        MatrixMxN(const std::array<VecN<N>, M>& initialValues) : rows(initialValues.begin(), initialValues.end()) {}

        // Copy constructor
        MatrixMxN(const MatrixMxN& other) : rows(other.rows) {}

        ~MatrixMxN() = default;

        // Zero the matrix
        void Zero() {
            for (auto& row : rows) {
                row.Zero();
            }
        }

        // Transpose the matrix
        MatrixMxN<N, M> Transpose() const {
            MatrixMxN<N, M> transposedMatrix;
            for (std::size_t i = 0; i < N; ++i) {
                for (std::size_t j = 0; j < M; ++j) {
                    transposedMatrix[i][j] = rows[j][i];
                }
            }
            return transposedMatrix;
        }

        // Assignment operator
        MatrixMxN& operator=(const MatrixMxN& other) {
            if (this != &other) {
                rows = other.rows;
            }
            return *this;
        }

        // Matrix-vector multiplication
        template<std::size_t P>
        VecN<N> operator*(const VecN<P>& v) const {
            static_assert(N == P, "Matrix and vector dimensions are not compatible for multiplication.");
            VecN<N> result;
            for (std::size_t i = 0; i < M; ++i) {
                result[i] = rows[i].Dot(v);
            }
            return result;
        }

        // Matrix-matrix multiplication with size checks
        template<std::size_t P, std::size_t Q>
        MatrixMxN<M, Q> operator*(const MatrixMxN<P, Q>& other) const {
            static_assert(N == P, "Matrix dimensions are not compatible for multiplication.");

            MatrixMxN<M, Q> result;
            for (std::size_t i = 0; i < M; ++i) {
                for (std::size_t j = 0; j < Q; ++j) {
                    for (std::size_t k = 0; k < N; ++k) {
                        result[i][j] += rows[i][k] * other[k][j];
                    }
                }
            }
            return result;
        }

        // Scalar multiplication
        MatrixMxN operator*(real scalar) const {
            MatrixMxN<M, N> result;
            for (std::size_t i = 0; i < M; ++i) {
                for (std::size_t j = 0; j < N; ++j) {
                    result[i][j] = rows[i][j] * scalar;
                }
            }
            return result;
        }

        // In-place scalar multiplication
        MatrixMxN& operator*=(real scalar) {
            for (auto& row : rows) {
                row *= scalar;
            }
            return *this;
        }

        // Access element (non-const and const)
        VecN<N>& operator[](std::size_t index) {
            if (index >= M) {
                throw std::out_of_range("Row index out of range");
            }
            return rows[index];
        }

        const VecN<N>& operator[](std::size_t index) const {
            if (index >= M) {
                throw std::out_of_range("Row index out of range");
            }
            return rows[index];
        }

        /// <summary>
        /// Static method for solving a system of linear equations using Gauss-Seidel method.
        /// Matrix A must be square, and vector b must have the same number of rows as A.
        /// </summary>
        /// <param name="A">The A matrix in Ax=b.</param>
        /// <param name="b">The b vector in Ax=b.</param>
        /// <returns></returns>
        static VecN<M> SolveGaussSeidel(const MatrixMxN<M, M>& A, const VecN<M>& b) {
            const int N = M;
            VecN<N> X;
            X.Zero();

            // Iterate N times
            for (int iterations = 0; iterations < N; iterations++) {
                for (int i = 0; i < N; i++) {
                    if (A.rows[i][i] != 0.0f) {
                        X[i] += (b[i] / A.rows[i][i]) - (A.rows[i].Dot(X) / A.rows[i][i]);
                    }
                }
            }

            return X;
        }
    };
}

#endif