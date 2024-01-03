#include "Matrix3x4.h"
#include <algorithm>
#include <iostream>

Aerolite::real Matrix3x4::CalcDeterminant()
{
    Aerolite::real subDet = data[0] * (data[5] * data[10] - data[6] * data[9]) -
                            data[1] * (data[4] * data[10] - data[6] * data[8]) +
                            data[2] * (data[4] * data[9] - data[5] * data[8]);

    return subDet;
}

Matrix3x4::Matrix3x4(
    Aerolite::real e0, Aerolite::real e1, Aerolite::real e2, Aerolite::real e3,
    Aerolite::real e4, Aerolite::real e5, Aerolite::real e6, Aerolite::real e7,
    Aerolite::real e8, Aerolite::real e9, Aerolite::real e10, Aerolite::real e11)
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
    data[9] = e9;
    data[10] = e10;
    data[11] = e11;

    determinant = CalcDeterminant();
}

Matrix3x4::Matrix3x4(Aerolite::real elements[12])
{
    std::copy(elements, elements + 12, data);
    determinant = CalcDeterminant();
}

void Matrix3x4::SetElement(int row, int column, Aerolite::real value)
{
    if (row >= 0 && row < 3 && column >= 0 && column < 4)
    {
        int index = row * 4 + column;
        data[index] = value;
        determinant = CalcDeterminant();
    }
    else
    {
        std::cerr << "Invalid row or column index. Row: " << row << ", Column: " << column << std::endl;
    }
}

Aerolite::Vec3 Matrix3x4::operator*(const Aerolite::Vec3& vector) const
{
    return Aerolite::Vec3(
        vector.x * data[0] + vector.y * data[1] + vector.z * data[2] + data[3],
        vector.x * data[4] + vector.y * data[5] + vector.z * data[6] + data[7],
        vector.x * data[8] + vector.y * data[9] + vector.z * data[10] + data[11]
    );
}

Matrix3x4 Matrix3x4::operator*(const Matrix3x4& o) const
{
    Matrix3x4 result;

    result.data[0] = o.data[0] * data[0] + o.data[4] * data[1] + o.data[8] * data[2];
    result.data[1] = o.data[1] * data[0] + o.data[5] * data[1] + o.data[9] * data[2];
    result.data[2] = o.data[2] * data[0] + o.data[6] * data[1] + o.data[10] * data[2];
    result.data[3] = o.data[3] * data[0] + o.data[7] * data[1] + o.data[11] * data[2] + data[3];

    result.data[4] = o.data[0] * data[4] + o.data[4] * data[5] + o.data[8] * data[6];
    result.data[5] = o.data[1] * data[4] + o.data[5] * data[5] + o.data[9] * data[6];
    result.data[6] = o.data[2] * data[4] + o.data[6] * data[5] + o.data[10] * data[6];
    result.data[7] = o.data[3] * data[4] + o.data[7] * data[5] + o.data[11] * data[6] + data[7];

    result.data[8] = o.data[0] * data[8] + o.data[4] * data[9] + o.data[8] * data[10];
    result.data[9] = o.data[1] * data[8] + o.data[5] * data[9] + o.data[9] * data[10];
    result.data[10] = o.data[2] * data[8] + o.data[6] * data[9] + o.data[10] * data[10];
    result.data[11] = o.data[3] * data[8] + o.data[7] * data[9] + o.data[11] * data[10] + data[11];

    return result;
}

Matrix3x4 Matrix3x4::MultMat3x4(const Matrix3x4& mat) const
{
    return (*this) * mat;
}

void Matrix3x4::operator*=(const Matrix3x4& o)
{
    Aerolite::real t1, t2, t3, t4;

    t1 = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
    t2 = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
    t3 = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];
    t4 = data[0] * o.data[3] + data[1] * o.data[7] + data[2] * o.data[11];
    
    data[0] = t1;
    data[1] = t2;
    data[2] = t3;
    data[3] = t4;

    t1 = data[4] * o.data[0] + data[5] * o.data[3] + data[6] * o.data[6];
    t2 = data[4] * o.data[1] + data[5] * o.data[4] + data[6] * o.data[7];
    t3 = data[4] * o.data[2] + data[5] * o.data[5] + data[6] * o.data[8];
    t4 = data[4] * o.data[3] + data[5] * o.data[7] + data[6] * o.data[11];
    
    data[4] = t1;
    data[5] = t2;
    data[6] = t3;
    data[7] = t4;

    t1 = data[8] * o.data[0] + data[9] * o.data[3] + data[10] * o.data[6];
    t2 = data[8] * o.data[1] + data[9] * o.data[4] + data[10] * o.data[7];
    t3 = data[8] * o.data[2] + data[9] * o.data[5] + data[10] * o.data[8];
    t4 = data[8] * o.data[3] + data[9] * o.data[7] + data[10] * o.data[11];
    
    data[8] = t1;
    data[9] = t2;
    data[10] = t3;
    data[11] = t4;

    determinant = CalcDeterminant();
}

Aerolite::Vec3 Matrix3x4::Transform(const Aerolite::Vec3& vector) const
{
    return (*this) * vector;
}

Aerolite::Vec3 Matrix3x4::TransformInverse(const Aerolite::Vec3& vector) const
{
    Aerolite::Vec3 tmp = vector; 
    tmp.x -= data[3];
    tmp.y -= data[7];
    tmp.z -= data[11]; 
    
    return Aerolite::Vec3(
    tmp.x * data[0] + tmp.y * data[4] + tmp.z * data[8],
    tmp.x * data[1] + tmp.y * data[5] + tmp.z * data[9],
    tmp.x * data[2] + tmp.y * data[6] + tmp.z * data[10]
    );
}

Aerolite::Vec3 Matrix3x4::TransformDirection(const Aerolite::Vec3& direction) const
{
    return Aerolite::Vec3(
        direction.x * data[0] +
        direction.y * data[1] + 
        direction.z * data[2],

        direction.x * data[4] + 
        direction.y * data[5] + 
        direction.z * data[6],

        direction.x * data[8] +
        direction.y * data[9] +
        direction.z * data[10]
    );
}

Aerolite::Vec3 Matrix3x4::TransformInverseDirection(const Aerolite::Vec3& direction) const
{
    return Aerolite::Vec3(
        direction.x * data[0] +
        direction.y * data[4] + 
        direction.z * data[8],

        direction.x * data[1] + 
        direction.y * data[5] + 
        direction.z * data[9],

        direction.x * data[2] +
        direction.y * data[6] +
        direction.z * data[10]
    );
}

bool Matrix3x4::SetInverse(const Matrix3x4& m)
{
    if (determinant != 0.0f)
    {
        Aerolite::real invDet = 1.0f / determinant;

        data[0] = (-m.data[9] * m.data[6] + m.data[5] * m.data[10]) * invDet;
        data[1] = (m.data[9] * m.data[2] - m.data[1] * m.data[10]) * invDet;
        data[2] = (-m.data[5] * m.data[2] + m.data[1] * m.data[6]) * invDet;

        data[4] = (m.data[8] * m.data[6] - m.data[4] * m.data[10]) * invDet;
        data[5] = (-m.data[8] * m.data[2] + m.data[0] * m.data[10]) * invDet;
        data[6] = (m.data[4] * m.data[2] - m.data[0] * m.data[6]) * invDet;

        data[8] = (-m.data[8] * m.data[5] + m.data[4] * m.data[9]) * invDet;
        data[9] = (m.data[8] * m.data[1] - m.data[0] * m.data[9]) * invDet;
        data[10] = (-m.data[4] * m.data[1] + m.data[0] * m.data[5]) * invDet;

        determinant = CalcDeterminant();

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

Aerolite::Vec3 Matrix3x4::LocalToWorld(const Aerolite::Vec3& local, const Matrix3x4& transform)
{
    return transform.Transform(local);
}


Aerolite::Vec3 Matrix3x4::WorldToLocal(const Aerolite::Vec3 &world, const Matrix3x4 &transform)
{
    return transform.TransformInverse(world);
}

Aerolite::Vec3 Matrix3x4::LocalToWorldDirection(const Aerolite::Vec3& localDirection, const Matrix3x4& transform)
{
    return transform.TransformDirection(localDirection);
}

Aerolite::Vec3 Matrix3x4::WorldToLocalDirection(const Aerolite::Vec3& worldDirection, const Matrix3x4& transform)
{
    return transform.TransformInverseDirection(worldDirection);
}


