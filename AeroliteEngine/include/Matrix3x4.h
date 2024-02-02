#ifndef MATRIX3X4_H
#define MATRIX3X4_H

#include "Precision.h"
#include "AeroVec3.h"

/// @brief Represents a 3x4 matrix with an implicit 4th row containg [0, 0, 0, 1] for computational effeciency.
/// Used to represent both a rotation and translation.
class Matrix3x4 
{
private:
    /// @brief Holds the matrix elements.
    Aerolite::real m_data[12];

    /// @brief The determinant of the matrix. 
    Aerolite::real m_determinant;

    /// @brief Calculates the determinant of the 3x3 sub-matrix in the upper-left corner.
    Aerolite::real CalcDeterminant();

public:
    Matrix3x4() = default;

    // Constructs a 3x4 matrix with each matrix element individually passed in.
    Matrix3x4(
        Aerolite::real e0, Aerolite::real e1, Aerolite::real e2, Aerolite::real e3,
        Aerolite::real e4, Aerolite::real e5, Aerolite::real e6, Aerolite::real e7,
        Aerolite::real e8, Aerolite::real e9, Aerolite::real e10, Aerolite::real e11);

    // Constructs a Matrix3x4 by copying the elements from the passed in array into its internal array structure.
    Matrix3x4(Aerolite::real elements[12]);

    /**
     * \brief Used to retrieve a specific element by row and column indices.
     * \param row The row of the element to retrieve.
     * \param column The column of the element to retrieve.
     * \return Returns the element at the given row/column indices in the matrix.
     */
    Aerolite::real GetElement(int row, int column);

    /// @brief Sets the specified element at the given row/column to the passed-in value.
    /// @param row The row to set the value at (0-based index).
    /// @param column The column to set the value at (0-based index).
    /// @param value The value to set the matrix element to.
    void SetElement(int row, int column, Aerolite::real value);

    /// @brief Operator overload for *. Performs both rotation and translations of the given vector by this matrix.
    /// @param vector the vector to transform.
    /// @return the transformed vector.
    Aerolite::AeroVec3 operator * (const Aerolite::AeroVec3& vector) const;

    /**
     * \brief Multiplies the given matrix o by this matrix.
     * \param o The matrix to be multiplied by this matrix.
     * \return Returns the result of this * o.
     */
    Matrix3x4 operator*(const Matrix3x4 &o) const;

    /**
     * \brief Performs matrix multiplication in the order of this * mat.
     * \param mat The matrix to be multiplied by this matrix.
     * \return Returns the result of this * mat.
     */
    Matrix3x4 MultMat3x4(const Matrix3x4& mat) const;

    /**
     * \brief Operator overload for *= which multiplies o by this matrix and stores the
     * result in this matrix.
     * \param o The matrix that will be multiplied by this matrix.
     */
    void operator*=(const Matrix3x4 &o);

    /**
     * \brief Overload operator[] to provide row access
     * \param row returns a pointer to the beginning of the row, which can be indexed again
     * to get the column.
     * \return Returns a pointer to the beginning of the row.
     */
    Aerolite::real* operator[](const int row) {
        // Assuming each row has 4 elements, calculate the index of the first element of the requested row
        return m_data + row * 4;
    }

    /**
    * \brief Constant version of the Overload operator[] to provide row access
    * \param row returns a pointer to the beginning of the row, which can be indexed again
    * to get the column.
    * \return Returns a pointer to the beginning of the row.
    */
    const Aerolite::real* operator[](const int row) const {
        return m_data + row * 4;
    }

    /// @brief Transforms the given vector by this matrix.
    /// @param vector The vector to transform.
    /// @return The transformed vector.
    Aerolite::AeroVec3 Transform(const Aerolite::AeroVec3& vector) const;

    /// @brief Transforms the given vector by the transformational inverse of this matrix.
    /// @param vector The vector to transform.
    /// @return The transformed vector.
    Aerolite::AeroVec3 TransformInverse(const Aerolite::AeroVec3& vector) const;

    /// @brief Transforms a direction vector by removing any translational component.
    /// Direction vectors should always remain pointing in the same direction, unless
    /// there is a rotational component to the transformation.
    /// @param direction The direction vector to transform.
    /// @return The transformed direction vector.
    Aerolite::AeroVec3 TransformDirection(const Aerolite::AeroVec3& direction) const;

    /// @brief Transform the direction vector by the transformational inverse of this matrix
    /// and removes any translational component. Direction vectors should always remain pointing in the same direction, unless
    /// there is a rotational component to the transformation.
    /// @param direction The direction vector to transform.
    /// @return The transformed direction vector.
    Aerolite::AeroVec3 TransformInverseDirection(const Aerolite::AeroVec3& direction) const;

    /// @brief Sets the current matrix to the inverse of the given matrix m.
    /// @param m The matrix to take the inverse of.
    /// @return Returns true if the matrix is invertible, and false if singular.
    bool SetInverse(const Matrix3x4& m);

    /// @brief Calculates the inverse of this matrix and returns it.
    /// @return The inverse of this matrix. If the matrix is singular, returns an identity matrix.
    Matrix3x4 Inverse() const;

    /// @brief Sets this matrix to the inverse of itself.
    /// @return Returns true if the matrix was inverted successfully, returns false if the matrix was singular.
    bool Invert();

    /// @brief Transform the given local vector by the given transform matrix to world coordinates.
    /// @param local The vector in local coordinates to transform.
    /// @param transform The 3x4 transform matrix to transform the local vector by.
    /// @return Returns the transformed AeroVec3.
    static Aerolite::AeroVec3 LocalToWorld(const Aerolite::AeroVec3& local, const Matrix3x4& transform);

    /// @brief Transform the given world vector by the given transformation to local coordinates.
    /// @param world The vector in world coordinates to transform to local coordinates.
    /// @param transform The 3x4 transform matrix to go from local to world coordinates.
    static Aerolite::AeroVec3 WorldToLocal(const Aerolite::AeroVec3& world, const Matrix3x4& transform);

    /// @brief Transforms the given local direction vector by given the world transform matrix with translation removed.
    /// @param localDirection The local direction vector to transform.
    /// @return Return the transformed direction vector.
    static Aerolite::AeroVec3 LocalToWorldDirection(const Aerolite::AeroVec3& localDirection, const Matrix3x4& transform);

    /// @brief Transforms the given world direction vector by given the local transform matrix with translation removed.
    /// @param worldDirection The world direction vector to transform.
    /// @return Return the transformed direction vector.
    static Aerolite::AeroVec3 WorldToLocalDirection(const Aerolite::AeroVec3& worldDirection, const Matrix3x4& transform);
};

#endif
