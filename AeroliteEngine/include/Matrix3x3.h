#ifndef MATRIX3X3_H
#define MATRIX3X3_H

#include "Precision.h"
#include "AeroVec3.h"

/**
 * \brief Defines a 3x3 matrix useful in rigid body calculations and various other
 * mathematical operations.
 */
class Matrix3x3
{
private:
    /// Holds the matrix elements.
    Aerolite::real m_data[9];
    /// The determinant of this matrix.
    Aerolite::real m_determinant;

    /// Private method to calculate the determinant of the matrix upon construction to increase efficiency.
    /// @return The determinant of the matrix.
    Aerolite::real CalcDeterminant() const;

public:
    Matrix3x3() = default;

    // Constructs a Matrix3x3 by copying the elements from the passed-in array into its internal array structure.
    Matrix3x3(Aerolite::real elements[9]);

    // Constructs a Matrix3x3 with the given elements passed in individually.
    Matrix3x3(
        Aerolite::real e0, Aerolite::real e1, Aerolite::real e2,
        Aerolite::real e3, Aerolite::real e4, Aerolite::real e5,
        Aerolite::real e6, Aerolite::real e7, Aerolite::real e8);

    /// Sets the specified element at the given row/column to the passed-in value.
    /// @param row The row to set the value at (0-based index).
    /// @param column The column to set the value at (0-based index).
    /// @param value The value to set the matrix element to.
    void SetElement(int row, int column, Aerolite::real value);

    /// Generates a 3x3 identity matrix.
    /// @return A 3x3 identity matrix.
    static Matrix3x3 Identity();

    // Transforms the given vector by this matrix.
    Aerolite::AeroVec3 operator*(const Aerolite::AeroVec3 &vector) const;

    /**
     * \brief Overload operator[] to provide row access
     * \param row returns a pointer to the beginning of the row, which can be indexed again
     * to get the column.
     * \return Returns a pointer to the beginning of the row.
     */
    Aerolite::real* operator[](const int row) {
        // Assuming each row has 4 elements, calculate the index of the first element of the requested row
        return m_data + row * 3;
    }

    /**
    * \brief Constant version of the Overload operator[] to provide row access
    * \param row returns a pointer to the beginning of the row, which can be indexed again
    * to get the column.
    * \return Returns a pointer to the beginning of the row.
    */
    const Aerolite::real* operator[](const int row) const {
        return m_data + row * 3;
    }

    /// Returns a matrix which is this one multiplied by the given matrix.
    /// @param o The matrix to multiply this matrix by.
    /// @return The resulting matrix after multiplication.
    Matrix3x3 operator*(const Matrix3x3 &o) const;

    /// @brief Sets this matrix to the transpose of the give matrix m.
    /// @param m The matrix to take the transpose of.
    void SetTranspose(Matrix3x3& m);

    /// @brief Returns the transpose of this matrix.
    /// @return The transpose of this matrix.
    Matrix3x3 Transpose(void);

    /**
     * \brief Returns the inverse of this matrix if the matrix is invertible.
     * Throws exception if matrix is invertible.
     * \return Returns the inverse of this matrix
     */
    Matrix3x3 Inverse() const;

    /**
     * \brief Sets this matrix to the inverse of the given matrix m.
     * \param m is the matrix to invert and then set this matrix to.
     */
    void SetInverse(const Matrix3x3& m);

    // Destructor
    ~Matrix3x3();
};

#endif
