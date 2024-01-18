#include <algorithm>
#include <cerrno>
#include <iostream>
#include "Matrix3x3.h"

// Constructor implementation that copies elements from an array.
Matrix3x3::Matrix3x3(Aerolite::real elements[9])
{
    std::copy(elements, elements + 9, data);
    determinant = CalcDeterminant();
}

// Constructor implementation that takes elements individually.
Matrix3x3::Matrix3x3(
    Aerolite::real e0, Aerolite::real e1, Aerolite::real e2,
    Aerolite::real e3, Aerolite::real e4, Aerolite::real e5,
    Aerolite::real e6, Aerolite::real e7, Aerolite::real e8)
{
    data[0] = e0;
    data[1] = e1;
    data[2] = e2;
    data[3] = e3;
    data[4] = e4;
    data[5] = e5;
    data[6] = e6;
    data[7] = e7;
    data[8] = e8;

    determinant = CalcDeterminant();
}

// Implementation of CalcDeterminant method.
Aerolite::real Matrix3x3::CalcDeterminant()
{
    Aerolite::real t1 = data[4] * data[8] - data[5] * data[7];
    Aerolite::real t2 = data[3] * data[8] - data[5] * data[6];
    Aerolite::real t3 = data[3] * data[7] - data[4] * data[6];

    return data[0] * t1 - data[1] * t2 + data[2] * t3;
}

// Sets the specified element at the given row/column to the passed-in value.
void Matrix3x3::SetElement(int row, int column, Aerolite::real value)
{
    // Check if the row and column indices are within the valid range (0 to 2 for both rows and columns).
    if (row >= 0 && row < 3 && column >= 0 && column < 3)
    {
        // Calculate the index in the 1D data array based on the row and column.
        int index = row * 3 + column;

        // Set the specified element to the passed-in value.
        data[index] = value;

        // Recalculate the determinant after modifying the matrix.
        determinant = CalcDeterminant();
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
Matrix3x3 Matrix3x3::Identity() const
{
    // Create a new Matrix3x3 and set its elements to form the identity matrix.
    return Matrix3x3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}

// Transforms the given vector by this matrix.
Aerolite::AeroVec3 Matrix3x3::operator*(const Aerolite::AeroVec3 &vector) const
{
    return Aerolite::AeroVec3(vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
                          vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
                          vector.x * data[6] + vector.y * data[7] + vector.z * data[8]);
}

// Returns a matrix which is this one multiplied by the given matrix.
Matrix3x3 Matrix3x3::operator*(const Matrix3x3 &o) const
{
    return Matrix3x3(
        data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
        data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
        data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

        data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
        data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
        data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

        data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
        data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
        data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]);
}

// Destructor
Matrix3x3::~Matrix3x3()
{
    
}

void Matrix3x3::SetTranspose(Matrix3x3& m)
{
    data[0] = m.data[0];
    data[1] = m.data[3]; 
    data[2] = m.data[6];
    data[3] = m.data[1];
    data[4] = m.data[4]; 
    data[5] = m.data[7];
    data[6] = m.data[2];
    data[7] = m.data[5];
    data[8] = m.data[8];
}

Matrix3x3 Matrix3x3::Transpose(void)
{
        Matrix3x3 result;
        result.SetTranspose(*this);
        return result;
}