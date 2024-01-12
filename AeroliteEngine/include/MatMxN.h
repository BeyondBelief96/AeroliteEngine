#ifndef MATRIXMN_H
#define MATRIXMN_H

#include "Precision.h"
#include "VecN.h"

namespace Aerolite {

    // The MatrixMN class template represents an M by N matrix.
    template <std::size_t M, std::size_t N>
    class MatrixMxN {
    private:
        // Holds the matrix elements.
        real data[M][N];

    public:
        // Default constructor. Initializes a new MatrixMN with default values.
        MatrixMxN() noexcept {
            for (std::size_t i = 0; i < M; ++i) {
                for (std::size_t j = 0; j < N; ++j) {
                    data[i][j] = real(0);
                }
            }
        }

        // Parameterized constructor. Initializes a new MatrixMN with specified elements.
        explicit MatrixMxN(const real elements[M][N]) noexcept {
            for (std::size_t i = 0; i < M; ++i) {
                for (std::size_t j = 0; j < N; ++j) {
                    data[i][j] = elements[i][j];
                }
            }
        }


        explicit MatrixMxN(const MatMN& m) {
            *this = m;
        }

        inline real Rows(void) const noexcept {
            return M;
        }

        inline real Columns(void) const noexcept {
            return N;
        }

        // Element access operator. Returns a reference to the element at the specified row and column.
        real& operator()(std::size_t row, std::size_t col) noexcept {
            return data[row][col];
        }

        // Element access operator (const version). Returns a const reference to the element at the specified row and column.
        const real& operator()(std::size_t row, std::size_t col) const noexcept {
            return data[row][col];
        }

        // Generates an identity matrix.
        static MatrixMxN Identity() noexcept {
            static_assert(M == N, "Identity matrix is only defined for square matrices.");
            MatrixMxN result;
            for (std::size_t i = 0; i < M; ++i) {
                result.data[i][i] = real(1);
            }
            return result;
        }

        // Transforms the given vector by this matrix.
        template <std::size_t P>
        VecN<P> operator*(const VecN<P>& vector) const noexcept {
            static_assert(N == P, "Matrix-vector multiplication requires the matrix column count to match the vector size.");
            VecN<P> result;
            for (std::size_t i = 0; i < M; ++i) {
                result(i) = real(0);
                for (std::size_t j = 0; j < N; ++j) {
                    result(i) += data[i][j] * vector(j);
                }
            }
            return result;
        }

        template <std::size_t M, std::size_t N>
        MatrixMxN<M, N>& operator=(const MatrixMxN<M, N>& other) noexcept {
            for (std::size_t i = 0; i < M; ++i) {
                for (std::size_t j = 0; j < N; ++j) {
                    data[i][j] = other(i, j);
                }
            }
            return *this;
        }

        // Matrix multiplication operator. Returns a new MatrixMN that is the result of multiplying this matrix by another matrix.
        template <std::size_t P>
        MatrixMxN<M, P> operator*(const MatrixMxN<N, P>& o) const noexcept {
            MatrixMxN<M, P> result;
            for (std::size_t i = 0; i < M; ++i) {
                for (std::size_t j = 0; j < P; ++j) {
                    result(i, j) = real(0);
                    for (std::size_t k = 0; k < N; ++k) {
                        result(i, j) += data[i][k] * o(k, j);
                    }
                }
            }
            return result;
        }

        // Returns the transpose of this matrix.
        MatrixMxN<N, M> Transpose() const noexcept {
            MatrixMxN<N, M> result;
            for (std::size_t i = 0; i < M; ++i) {
                for (std::size_t j = 0; j < N; ++j) {
                    result(j, i) = data[i][j];
                }
            }
            return result;
        }

        // Matrix multiplication function for MatrixMN multiplied by another MatrixMN
        template <std::size_t M, std::size_t N, std::size_t P>
        static MatrixMxN<M, P> MatrixMultiply(const MatrixMxN<M, N>& a, const MatrixMxN<N, P>& b) {
            if (N != a.Columns() || N != b.Rows()) {
                throw std::invalid_argument("Matrix sizes are incompatible for multiplication.");
            }

            MatrixMxN<M, P> result;
            for (std::size_t i = 0; i < M; ++i) {
                for (std::size_t j = 0; j < P; ++j) {
                    result(i, j) = real(0);
                    for (std::size_t k = 0; k < N; ++k) {
                        result(i, j) += a(i, k) * b(k, j);
                    }
                }
            }
            return result;
        }

        // Destructor
        ~MatrixMxN() noexcept = default;
    };
}

#endif // MATRIXMN_H
