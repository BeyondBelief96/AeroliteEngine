#include <algorithm>
#include <iostream>
#include "Matrix3x3.h"

// Destructor
Matrix3x3::~Matrix3x3()
{

}

// Constructor implementation that copies elements from an array.
Matrix3x3::Matrix3x3(Aerolite::real elements[9])
{
    std::copy(elements, elements + 9, m_data);
    m_determinant = CalcDeterminant();
}

// Constructor implementation that takes elements individually.
Matrix3x3::Matrix3x3(
	const Aerolite::real e0, const Aerolite::real e1, const Aerolite::real e2,
	const Aerolite::real e3, const Aerolite::real e4, const Aerolite::real e5,
	const Aerolite::real e6, const Aerolite::real e7, const Aerolite::real e8)
{
    m_data[0] = e0;
    m_data[1] = e1;
    m_data[2] = e2;
    m_data[3] = e3;
    m_data[4] = e4;
    m_data[5] = e5;
    m_data[6] = e6;
    m_data[7] = e7;
    m_data[8] = e8;

    m_determinant = CalcDeterminant();
}

// Implementation of CalcDeterminant method.
Aerolite::real Matrix3x3::CalcDeterminant() const
{
    const Aerolite::real t1 = m_data[4] * m_data[8] - m_data[5] * m_data[7];
    const Aerolite::real t2 = m_data[3] * m_data[8] - m_data[5] * m_data[6];
    const Aerolite::real t3 = m_data[3] * m_data[7] - m_data[4] * m_data[6];

    return m_data[0] * t1 - m_data[1] * t2 + m_data[2] * t3;
}

// Sets the specified element at the given row/column to the passed-in value.
void Matrix3x3::SetElement(const int row, const int column, const Aerolite::real value)
{
    // Check if the row and column indices are within the valid range (0 to 2 for both rows and columns).
    if (row >= 0 && row < 3 && column >= 0 && column < 3)
    {
        // Calculate the index in the 1D data array based on the row and column.
        int index = row * 3 + column;

        // Set the specified element to the passed-in value.
        m_data[index] = value;

        // Recalculate the determinant after modifying the matrix.
        m_determinant = CalcDeterminant();
    }
    else
    {
        // Handle an invalid row or column index here (e.g., print an error message or throw an exception).
        // You can add error handling based on your requirements.
        // For now, let's print an error message to the console.
        std::cerr << "Invalid row or column index. Row: " << row << ", Column: " << column << std::endl;
    }
}

// Generates a 3x3 identity matrix.
Matrix3x3 Matrix3x3::Identity()
{
    // Create a new Matrix3x3 and set its elements to form the identity matrix.
    return Matrix3x3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}

// Transforms the given vector by this matrix.
Aerolite::AeroVec3 Matrix3x3::operator*(const Aerolite::AeroVec3 &vector) const
{
    return Aerolite::AeroVec3(vector.x * m_data[0] + vector.y * m_data[1] + vector.z * m_data[2],
                          vector.x * m_data[3] + vector.y * m_data[4] + vector.z * m_data[5],
                          vector.x * m_data[6] + vector.y * m_data[7] + vector.z * m_data[8]);
}

// Returns a matrix which is this one multiplied by the given matrix.
Matrix3x3 Matrix3x3::operator*(const Matrix3x3 &o) const
{
    return Matrix3x3(
        m_data[0] * o.m_data[0] + m_data[1] * o.m_data[3] + m_data[2] * o.m_data[6],
        m_data[0] * o.m_data[1] + m_data[1] * o.m_data[4] + m_data[2] * o.m_data[7],
        m_data[0] * o.m_data[2] + m_data[1] * o.m_data[5] + m_data[2] * o.m_data[8],

        m_data[3] * o.m_data[0] + m_data[4] * o.m_data[3] + m_data[5] * o.m_data[6],
        m_data[3] * o.m_data[1] + m_data[4] * o.m_data[4] + m_data[5] * o.m_data[7],
        m_data[3] * o.m_data[2] + m_data[4] * o.m_data[5] + m_data[5] * o.m_data[8],

        m_data[6] * o.m_data[0] + m_data[7] * o.m_data[3] + m_data[8] * o.m_data[6],
        m_data[6] * o.m_data[1] + m_data[7] * o.m_data[4] + m_data[8] * o.m_data[7],
        m_data[6] * o.m_data[2] + m_data[7] * o.m_data[5] + m_data[8] * o.m_data[8]);
}


void Matrix3x3::SetTranspose(Matrix3x3& m)
{
    m_data[0] = m.m_data[0];
    m_data[1] = m.m_data[3]; 
    m_data[2] = m.m_data[6];
    m_data[3] = m.m_data[1];
    m_data[4] = m.m_data[4]; 
    m_data[5] = m.m_data[7];
    m_data[6] = m.m_data[2];
    m_data[7] = m.m_data[5];
    m_data[8] = m.m_data[8];
}

Matrix3x3 Matrix3x3::Transpose(void)
{
        Matrix3x3 result;
        result.SetTranspose(*this);
        return result;
}

Matrix3x3 Matrix3x3::Inverse() const
{
    const Aerolite::real det = CalcDeterminant();
    if (Aerolite::AreEqual(det, 0, Aerolite::EPSILON)) {
        throw std::runtime_error("Matrix is singular and cannot be inverted.");
    }

    Matrix3x3 inverse;
    // Calculate inverse using the formula for 3x3 matrix
    inverse.SetElement(0, 0, (m_data[4] * m_data[8] - m_data[5] * m_data[7]) / det);
    inverse.SetElement(0, 1, -(m_data[1] * m_data[8] - m_data[2] * m_data[7]) / det);
    inverse.SetElement(0, 2, (m_data[1] * m_data[5] - m_data[2] * m_data[4]) / det);
    inverse.SetElement(1, 0, -(m_data[3] * m_data[8] - m_data[5] * m_data[6]) / det);
    inverse.SetElement(1, 1, (m_data[0] * m_data[8] - m_data[2] * m_data[6]) / det);
    inverse.SetElement(1, 2, -(m_data[0] * m_data[5] - m_data[2] * m_data[3]) / det);
    inverse.SetElement(2, 0, (m_data[3] * m_data[7] - m_data[4] * m_data[6]) / det);
    inverse.SetElement(2, 1, -(m_data[0] * m_data[7] - m_data[1] * m_data[6]) / det);
    inverse.SetElement(2, 2, (m_data[0] * m_data[4] - m_data[1] * m_data[3]) / det);

    return inverse;
}

void Matrix3x3::SetInverse(const Matrix3x3& m)
{
    const Aerolite::real det = m.CalcDeterminant();
    if (Aerolite::AreEqual(det, 0, Aerolite::EPSILON)) {
        throw std::runtime_error("Matrix is singular and cannot be inverted.");
    }

    // Set this matrix to the inverse of matrix m
    SetElement(0, 0, (m.m_data[4] * m.m_data[8] - m.m_data[5] * m.m_data[7]) / det);
    SetElement(0, 1, -(m.m_data[1] * m.m_data[8] - m.m_data[2] * m.m_data[7]) / det);
    SetElement(0, 2, (m.m_data[1] * m.m_data[5] - m.m_data[2] * m.m_data[4]) / det);
    SetElement(1, 0, -(m.m_data[3] * m.m_data[8] - m.m_data[5] * m.m_data[6]) / det);
    SetElement(1, 1, (m.m_data[0] * m.m_data[8] - m.m_data[2] * m.m_data[6]) / det);
    SetElement(1, 2, -(m.m_data[0] * m.m_data[5] - m.m_data[2] * m.m_data[3]) / det);
    SetElement(2, 0, (m.m_data[3] * m.m_data[7] - m.m_data[4] * m.m_data[6]) / det);
    SetElement(2, 1, -(m.m_data[0] * m.m_data[7] - m.m_data[1] * m.m_data[6]) / det);
    SetElement(2, 2, (m.m_data[0] * m.m_data[4] - m.m_data[1] * m.m_data[3]) / det);
}
