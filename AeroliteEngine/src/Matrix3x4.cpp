#include "Matrix3x4.h"
#include <algorithm>
#include <iostream>

Aerolite::real Matrix3x4::CalcDeterminant()
{
    Aerolite::real subDet = m_data[0] * (m_data[5] * m_data[10] - m_data[6] * m_data[9]) -
                            m_data[1] * (m_data[4] * m_data[10] - m_data[6] * m_data[8]) +
                            m_data[2] * (m_data[4] * m_data[9] - m_data[5] * m_data[8]);

    return subDet;
}

Matrix3x4::Matrix3x4(
    Aerolite::real e0, Aerolite::real e1, Aerolite::real e2, Aerolite::real e3,
    Aerolite::real e4, Aerolite::real e5, Aerolite::real e6, Aerolite::real e7,
    Aerolite::real e8, Aerolite::real e9, Aerolite::real e10, Aerolite::real e11)
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
    m_data[9] = e9;
    m_data[10] = e10;
    m_data[11] = e11;

    m_determinant = CalcDeterminant();
}

Matrix3x4::Matrix3x4(Aerolite::real elements[12])
{
    std::copy(elements, elements + 12, m_data);
    m_determinant = CalcDeterminant();
}

Aerolite::real Matrix3x4::GetElement(const int row, const int column)
{
    return m_data[(row * 4) + column];
}

void Matrix3x4::SetElement(int row, int column, Aerolite::real value)
{
    if (row >= 0 && row < 3 && column >= 0 && column < 4)
    {
        int index = row * 4 + column;
        m_data[index] = value;
        m_determinant = CalcDeterminant();
    }
    else
    {
        std::cerr << "Invalid row or column index. Row: " << row << ", Column: " << column << std::endl;
    }
}

Aerolite::AeroVec3 Matrix3x4::operator*(const Aerolite::AeroVec3& vector) const
{
    return Aerolite::AeroVec3(
        vector.x * m_data[0] + vector.y * m_data[1] + vector.z * m_data[2] + m_data[3],
        vector.x * m_data[4] + vector.y * m_data[5] + vector.z * m_data[6] + m_data[7],
        vector.x * m_data[8] + vector.y * m_data[9] + vector.z * m_data[10] + m_data[11]
    );
}

Matrix3x4 Matrix3x4::operator*(const Matrix3x4& o) const
{
    Matrix3x4 result;

    result.m_data[0] = o.m_data[0] * m_data[0] + o.m_data[4] * m_data[1] + o.m_data[8] * m_data[2];
    result.m_data[1] = o.m_data[1] * m_data[0] + o.m_data[5] * m_data[1] + o.m_data[9] * m_data[2];
    result.m_data[2] = o.m_data[2] * m_data[0] + o.m_data[6] * m_data[1] + o.m_data[10] * m_data[2];
    result.m_data[3] = o.m_data[3] * m_data[0] + o.m_data[7] * m_data[1] + o.m_data[11] * m_data[2] + m_data[3];

    result.m_data[4] = o.m_data[0] * m_data[4] + o.m_data[4] * m_data[5] + o.m_data[8] * m_data[6];
    result.m_data[5] = o.m_data[1] * m_data[4] + o.m_data[5] * m_data[5] + o.m_data[9] * m_data[6];
    result.m_data[6] = o.m_data[2] * m_data[4] + o.m_data[6] * m_data[5] + o.m_data[10] * m_data[6];
    result.m_data[7] = o.m_data[3] * m_data[4] + o.m_data[7] * m_data[5] + o.m_data[11] * m_data[6] + m_data[7];

    result.m_data[8] = o.m_data[0] * m_data[8] + o.m_data[4] * m_data[9] + o.m_data[8] * m_data[10];
    result.m_data[9] = o.m_data[1] * m_data[8] + o.m_data[5] * m_data[9] + o.m_data[9] * m_data[10];
    result.m_data[10] = o.m_data[2] * m_data[8] + o.m_data[6] * m_data[9] + o.m_data[10] * m_data[10];
    result.m_data[11] = o.m_data[3] * m_data[8] + o.m_data[7] * m_data[9] + o.m_data[11] * m_data[10] + m_data[11];

    return result;
}

Matrix3x4 Matrix3x4::MultMat3x4(const Matrix3x4& mat) const
{
    return (*this) * mat;
}

void Matrix3x4::operator*=(const Matrix3x4& o)
{
    Aerolite::real t1, t2, t3, t4;

    t1 = m_data[0] * o.m_data[0] + m_data[1] * o.m_data[3] + m_data[2] * o.m_data[6];
    t2 = m_data[0] * o.m_data[1] + m_data[1] * o.m_data[4] + m_data[2] * o.m_data[7];
    t3 = m_data[0] * o.m_data[2] + m_data[1] * o.m_data[5] + m_data[2] * o.m_data[8];
    t4 = m_data[0] * o.m_data[3] + m_data[1] * o.m_data[7] + m_data[2] * o.m_data[11];
    
    m_data[0] = t1;
    m_data[1] = t2;
    m_data[2] = t3;
    m_data[3] = t4;

    t1 = m_data[4] * o.m_data[0] + m_data[5] * o.m_data[3] + m_data[6] * o.m_data[6];
    t2 = m_data[4] * o.m_data[1] + m_data[5] * o.m_data[4] + m_data[6] * o.m_data[7];
    t3 = m_data[4] * o.m_data[2] + m_data[5] * o.m_data[5] + m_data[6] * o.m_data[8];
    t4 = m_data[4] * o.m_data[3] + m_data[5] * o.m_data[7] + m_data[6] * o.m_data[11];
    
    m_data[4] = t1;
    m_data[5] = t2;
    m_data[6] = t3;
    m_data[7] = t4;

    t1 = m_data[8] * o.m_data[0] + m_data[9] * o.m_data[3] + m_data[10] * o.m_data[6];
    t2 = m_data[8] * o.m_data[1] + m_data[9] * o.m_data[4] + m_data[10] * o.m_data[7];
    t3 = m_data[8] * o.m_data[2] + m_data[9] * o.m_data[5] + m_data[10] * o.m_data[8];
    t4 = m_data[8] * o.m_data[3] + m_data[9] * o.m_data[7] + m_data[10] * o.m_data[11];
    
    m_data[8] = t1;
    m_data[9] = t2;
    m_data[10] = t3;
    m_data[11] = t4;

    m_determinant = CalcDeterminant();
}

Aerolite::AeroVec3 Matrix3x4::Transform(const Aerolite::AeroVec3& vector) const
{
    return (*this) * vector;
}

Aerolite::AeroVec3 Matrix3x4::TransformInverse(const Aerolite::AeroVec3& vector) const
{
    Aerolite::AeroVec3 tmp;
    tmp = vector;
    tmp.x -= m_data[3];
    tmp.y -= m_data[7];
    tmp.z -= m_data[11]; 
    
    return Aerolite::AeroVec3(
    tmp.x * m_data[0] + tmp.y * m_data[4] + tmp.z * m_data[8],
    tmp.x * m_data[1] + tmp.y * m_data[5] + tmp.z * m_data[9],
    tmp.x * m_data[2] + tmp.y * m_data[6] + tmp.z * m_data[10]
    );
}

Aerolite::AeroVec3 Matrix3x4::TransformDirection(const Aerolite::AeroVec3& direction) const
{
    return Aerolite::AeroVec3(
        direction.x * m_data[0] +
        direction.y * m_data[1] + 
        direction.z * m_data[2],

        direction.x * m_data[4] + 
        direction.y * m_data[5] + 
        direction.z * m_data[6],

        direction.x * m_data[8] +
        direction.y * m_data[9] +
        direction.z * m_data[10]
    );
}

Aerolite::AeroVec3 Matrix3x4::TransformInverseDirection(const Aerolite::AeroVec3& direction) const
{
    return Aerolite::AeroVec3(
        direction.x * m_data[0] +
        direction.y * m_data[4] + 
        direction.z * m_data[8],

        direction.x * m_data[1] + 
        direction.y * m_data[5] + 
        direction.z * m_data[9],

        direction.x * m_data[2] +
        direction.y * m_data[6] +
        direction.z * m_data[10]
    );
}

bool Matrix3x4::SetInverse(const Matrix3x4& m)
{
    if (m_determinant != 0.0f)
    {
        Aerolite::real invDet = 1.0f / m_determinant;

        m_data[0] = (-m.m_data[9] * m.m_data[6] + m.m_data[5] * m.m_data[10]) * invDet;
        m_data[1] = (m.m_data[9] * m.m_data[2] - m.m_data[1] * m.m_data[10]) * invDet;
        m_data[2] = (-m.m_data[5] * m.m_data[2] + m.m_data[1] * m.m_data[6]) * invDet;

        m_data[4] = (m.m_data[8] * m.m_data[6] - m.m_data[4] * m.m_data[10]) * invDet;
        m_data[5] = (-m.m_data[8] * m.m_data[2] + m.m_data[0] * m.m_data[10]) * invDet;
        m_data[6] = (m.m_data[4] * m.m_data[2] - m.m_data[0] * m.m_data[6]) * invDet;

        m_data[8] = (-m.m_data[8] * m.m_data[5] + m.m_data[4] * m.m_data[9]) * invDet;
        m_data[9] = (m.m_data[8] * m.m_data[1] - m.m_data[0] * m.m_data[9]) * invDet;
        m_data[10] = (-m.m_data[4] * m.m_data[1] + m.m_data[0] * m.m_data[5]) * invDet;

        m_determinant = CalcDeterminant();

        return true;
    }
    else
    {
        std::cerr << "Matrix is singular, and its inverse does not exist." << std::endl;
        return false;
    }
}

Matrix3x4 Matrix3x4::Inverse() const
{
    Matrix3x4 result;

    if (result.SetInverse(*this))
    {
        return result;
    }
    else
    {
        std::cerr << "Matrix is singular, and its inverse does not exist." << std::endl;
        // Return an identity matrix as a fallback.
        return Matrix3x4(1.0f, 0.0f, 0.0f, 0.0f,
                         0.0f, 1.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 1.0f, 0.0f);
    }
}

bool Matrix3x4::Invert()
{
    return SetInverse(*this);
}

Aerolite::AeroVec3 Matrix3x4::LocalToWorld(const Aerolite::AeroVec3& local, const Matrix3x4& transform)
{
    return transform.Transform(local);
}


Aerolite::AeroVec3 Matrix3x4::WorldToLocal(const Aerolite::AeroVec3 &world, const Matrix3x4 &transform)
{
    return transform.TransformInverse(world);
}

Aerolite::AeroVec3 Matrix3x4::LocalToWorldDirection(const Aerolite::AeroVec3& localDirection, const Matrix3x4& transform)
{
    return transform.TransformDirection(localDirection);
}

Aerolite::AeroVec3 Matrix3x4::WorldToLocalDirection(const Aerolite::AeroVec3& worldDirection, const Matrix3x4& transform)
{
    return transform.TransformInverseDirection(worldDirection);
}


