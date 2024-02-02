#include "AeroBody3D.h"

namespace Aerolite
{
	static inline void CalculateTransformMatrix(Matrix3x4& transformMatrix,
		const AeroVec3& position, const Quaternion& orientation)
	{
		transformMatrix.SetElement(0, 0, 1 - 2 * orientation.j * orientation.j - 2 * orientation.k * orientation.k);
		transformMatrix.SetElement(0, 1, 2 * orientation.i * orientation.j - 2 * orientation.r * orientation.k);
		transformMatrix.SetElement(0, 2, 2 * orientation.i * orientation.k + 2 * orientation.r * orientation.j);
		transformMatrix.SetElement(0, 3, position.x);

		transformMatrix.SetElement(1, 0, 2 * orientation.i * orientation.j + 2 * orientation.r * orientation.k);
		transformMatrix.SetElement(1, 1, 1 - 2 * orientation.i * orientation.i - 2 * orientation.k * orientation.k);
		transformMatrix.SetElement(1, 2, 2 * orientation.j * orientation.k - 2 * orientation.r * orientation.i);
		transformMatrix.SetElement(1, 3, position.y);

		transformMatrix.SetElement(2, 0, 2 * orientation.i * orientation.k - 2 * orientation.r * orientation.j);
		transformMatrix.SetElement(2, 1, 2 * orientation.j * orientation.k + 2 * orientation.r * orientation.i);
		transformMatrix.SetElement(2, 2, 1 - 2 * orientation.i * orientation.i - 2 * orientation.j * orientation.j);
		transformMatrix.SetElement(2, 3, position.z);
	}

    /**
    * @brief Transforms the inertia tensor from body coordinates to world coordinates.
    * @param iitWorld Reference to the inertia tensor in world coordinates to be updated.
    * @param q The rotation quaternion.
    * @param iitBody The inertia tensor in body coordinates.
    * @param rotmat The rotation matrix corresponding to the quaternion.
    */
    static inline void TransformInertiaTensorLocalToWorld(
        Matrix3x3& iitWorld,
        const Quaternion& q,
        const Matrix3x3& iitBody,
        const Matrix3x4& rotmat) {

        // First row of the transformed inertia tensor
        const real t4 = rotmat[0][0] * iitBody[0][0] + rotmat[0][1] * iitBody[0][1] + rotmat[0][2] * iitBody[0][2];
        const real t9 = rotmat[0][0] * iitBody[1][0] + rotmat[0][1] * iitBody[1][1] + rotmat[0][2] * iitBody[1][2];
        const real t14 = rotmat[0][0] * iitBody[2][0] + rotmat[0][1] * iitBody[2][1] + rotmat[0][2] * iitBody[2][2];
        iitWorld[0][0] = t4 * rotmat[0][0] + t9 * rotmat[0][1] + t14 * rotmat[0][2];
        iitWorld[0][1] = t4 * rotmat[1][0] + t9 * rotmat[1][1] + t14 * rotmat[1][2];
        iitWorld[0][2] = t4 * rotmat[2][0] + t9 * rotmat[2][1] + t14 * rotmat[2][2];

        // Second row of the transformed inertia tensor
        const real t28 = rotmat[1][0] * iitBody[0][0] + rotmat[1][1] * iitBody[0][1] + rotmat[1][2] * iitBody[0][2];
        const real t33 = rotmat[1][0] * iitBody[1][0] + rotmat[1][1] * iitBody[1][1] + rotmat[1][2] * iitBody[1][2];
        const real t38 = rotmat[1][0] * iitBody[2][0] + rotmat[1][1] * iitBody[2][1] + rotmat[1][2] * iitBody[2][2];
        iitWorld[1][0] = t28 * rotmat[0][0] + t33 * rotmat[0][1] + t38 * rotmat[0][2];
        iitWorld[1][1] = t28 * rotmat[1][0] + t33 * rotmat[1][1] + t38 * rotmat[1][2];
        iitWorld[1][2] = t28 * rotmat[2][0] + t33 * rotmat[2][1] + t38 * rotmat[2][2];

        // Third row of the transformed inertia tensor
        const real t52 = rotmat[2][0] * iitBody[0][0] + rotmat[2][1] * iitBody[0][1] + rotmat[2][2] * iitBody[0][2];
        const real t57 = rotmat[2][0] * iitBody[1][0] + rotmat[2][1] * iitBody[1][1] + rotmat[2][2] * iitBody[1][2];
        const real t62 = rotmat[2][0] * iitBody[2][0] + rotmat[2][1] * iitBody[2][1] + rotmat[2][2] * iitBody[2][2];
        iitWorld[2][0] = t52 * rotmat[0][0] + t57 * rotmat[0][1] + t62 * rotmat[0][2];
        iitWorld[2][1] = t52 * rotmat[1][0] + t57 * rotmat[1][1] + t62 * rotmat[1][2];
        iitWorld[2][2] = t52 * rotmat[2][0] + t57 * rotmat[2][1] + t62 * rotmat[2][2];
    }

    bool AeroBody3D::IsStatic() const
    {
        return AreEqual(mass, 0.0, EPSILON);
    }

	void AeroBody3D::CalculateDerivedData()
	{
		orientation.Normalize();
		CalculateTransformMatrix(transformation_matrix, position, orientation);
        TransformInertiaTensorLocalToWorld(inverse_inertia_tensor_world, orientation,
            inverse_intertia_tensor_local, transformation_matrix);
	}

	void AeroBody3D::SetInertiaTensor(const Matrix3x3& inertiaTensor)
	{
		inverse_intertia_tensor_local.SetInverse(inertiaTensor);
	}

	void AeroBody3D::AddForce(const AeroVec3& force)
	{
        sum_forces += force;
        is_awake = true;
	}

	void AeroBody3D::AddForceAtWorldPoint(const AeroVec3& force, const AeroVec3& point)
	{
        AeroVec3 pt = point;
        pt -= position;

        sum_forces += force;
        sum_torques += pt.Cross(force);
        is_awake = true;
	}

	void AeroBody3D::AddForceAtBodyPoint(const AeroVec3& force, const AeroVec3& point)
	{
        const AeroVec3 pt = GetPointInWorldSpace(point);
        AddForceAtWorldPoint(force, pt);
	}

    void AeroBody3D::SetPosition(const AeroVec3& p)
    {
        this->position = p;
    }

    void AeroBody3D::SetPosition(const real x, const real y, const real z)
    {
        position.x = x;
        position.y = y;
        position.z = z;
    }

    AeroVec3 AeroBody3D::GetPointInLocalSpace(const AeroVec3& point) const
    {
        return transformation_matrix.TransformInverse(point);
    }

    AeroVec3 AeroBody3D::GetPointInWorldSpace(const AeroVec3& point) const
    {
        return transformation_matrix.Transform(point);
    }

	void AeroBody3D::ClearForces()
	{
        sum_forces *= 0;
	}

	void AeroBody3D::ClearTorque()
	{
        sum_torques *= 0;
	}
}

