#ifndef MATRIX3X3_H
#define MATRIX3X3_H

#include "Precision.h"
#include "AeroVec3.h"

class Matrix3x3
{
private:
    /// Holds the matrix elements.
    Aerolite::real data[9];
    /// The determinant of this matrix.
    Aerolite::real determinant;

    /// Private method to calculate the determinant of the matrix upon construction to increase efficiency.
    /// @return The determinant of the matrix.
    Aerolite::real CalcDeterminant();

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
    Matrix3x3 Identity() const;

    // Transforms the given vector by this matrix.
    Aerolite::AeroVec3 operator*(const Aerolite::AeroVec3 &vector) const;

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

    // Destructor
    ~Matrix3x3();
};

#endif
