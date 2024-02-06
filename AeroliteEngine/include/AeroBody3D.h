#ifndef AEROBODY_3D_H
#define AEROBODY_3D_h

#include "AeroQuat.h"
#include "AeroVec3.h"
#include "Matrix3x3.h"
#include "Matrix3x4.h"
#include "Shape.h"

namespace Aerolite
{
	/**
	* \brief Represents a 3-Dimensional rigid body. This is the basic simulation
	* object for the 3D portion of the physics engine.
	*/
	struct AeroBody3D
	{
		AeroBody3D(Shape* shape, const real x, const real y, const real z, const real mass);

		~AeroBody3D();

		/**
		 * \brief Position of the rigid bodies center of mass in 3D space.
		 */
		AeroVec3 position;

		/**
		 * \brief  Linear velocity of the rigid bodies center of mass.
		 */
		AeroVec3 velocity;

		/**
		 * \brief Linear acceleration of the rigid bodies center of mass.
		 */
		AeroVec3 acceleration;

		/**
		 * \brief holds the angular orientation of the rigid body in world space.
		 */
		Quaternion orientation;

		/**
		 * \brief Holds the angular velocity of the rigid body in world space.
		 */
		AeroVec3 angular_velocity;

		/**
		 * \brief Holds the angular acceleration of the rigid body in world space.
		 */
		AeroVec3 angular_acceleration;

		/**
		 * \brief Holds a transformation matrix for converting between body space into
		 * world space for this rigid body and vice versa.
		 */
		Matrix3x4 transformation_matrix;

		/**
		 * \brief Sum of all forces currently being applied to the rigid body.
		 */
		AeroVec3 sum_forces;

		/**
		 * \brief Sum of all torques currently being applied to the rigid body.
		 */
		AeroVec3 sum_torques;

		/**
		 * \brief Holds the amount of linear damping to be applied during motion.
		 * Removes energy added through numerical instability of the integrator.
		 */
		real linear_damping;

		/**
		 * \brief Holds the amount of angular damping to be applied during motion.
		 * Removes energy added through numerical instability of the integrator.
		 */
		real angular_damping;

		/**
		 * \brief The mass of the rigid body. Initialized to 0.
		 */
		real mass = 0.0f;

		/**
		 * \brief Used to cause certain rigid bodies to "sleep" after no significant
		 * forces have been applied after some threshold of time. Sleeping bodies
		 * won't get updated in the integrator.
		 */
		bool is_awake;

		/**
		 * \brief Holds the inverse of the body's inertia tensor.
		 * The inertia tensor provided must not be degenerate
		 * (meaning the body had zero inertia for spinning along one axis).
		 * As long as the tensor is finite, it will be invertible. The inverse inertia
		 * tensor is used for similar reasons we use the inverse mass.
		 *
		 * The inverse inertia tensor is given in body space instead of world space.
		 */
		Matrix3x3 inverse_intertia_tensor_local;

		/**
		 * \brief Holds the inverse inertia tensor of the rigid body
		 * in world coordinates. This is needed due to the calculation of
		 * the angular velocity being done in world coordinates. This will be
		 * calculated every frame to ensure it stays up to date.
		 */
		Matrix3x3 inverse_inertia_tensor_world;

		/**
		 * \brief Inverse of the mass. Useful for numerical calculations to avoid dividing
		 * by floating point numbers.
		 */
		real inverse_mass = 0.0f;

		/**
		 * \brief Coefficient of restitution of the rigid body.
		 */
		real restitution;

		/**
		 * \brief The shape of the rigid body. Initialized to nullptr.
		 */
		Shape* shape = nullptr;

		/**
		* \brief Determines if the AeroBody2D has an infinite mass.
		* \return Returns true if mass is zero, false otherwise.
		*/
		bool IsStatic(void) const;

		/**
		 * \brief Calculates internal data from state data. This should be called
		 * after the body's state is altered directly (such as during integration).
		 * If you change the body's state, and then intend to integrate before querying
		 * any data (such as the transformation matrix), then you can omit this step. 
		 **/
		void CalculateDerivedData();

		/**
		 * \brief Takes the inverse of the given inertia tensor and sets this rigid bodies
		 * inverse inertia tensor.
		 * \param inertiaTensor The inertia tensor to take the inverse of
		 */
		void SetInertiaTensor(const Matrix3x3& inertiaTensor);

		/**
		 * \brief Adds the given force to the body.
		 * \param force The force to add to this body (applied at the center of mass).
		 * The force is expected to be expressed in world coordinates.
		 */
		void AddForce(const AeroVec3& force);

		/**
		* \brief Clears the accumulated forces to zero.
		*/
		void ClearForces(void);

		/**
		* \brief Clears the accumulated torques to zero.
		*/
		void ClearTorque(void);

		/**
		* \brief Performs integration of the forces/torques and linear/angular accelerations to find the new velocities of the current simulation frame.
		*/
		void IntegrateForces(const real dt);

		/**
		* \brief Performs integration of the linear and angular velocities to find the new position/rotation of the current simulation frame.
		*/
		void IntegrateVelocities(const real dt);

		/**
		* \brief Applies a linear impulse to the body at the center of mass.
		*/
		void ApplyImpulseLinear(const AeroVec3& j);

		/**
		* \brief Applies an angular impulse to the body at the center of mass.
		*/
		void ApplyImpulseAngular(const real j);

		/**
		* \brief Applies an impulse j to the body at the point r generated from a collision.
		*/
		void ApplyImpulseAtPoint(const AeroVec3& j, const AeroVec3& r);

		/**
		 * \brief Adds the given force to the given point on the rigid body.
		 * \param force The force to apply given in world coordinates.
		 * \param point The point on the rigid body to apply the force to
		 * given in world coordinates. This function may result in a torque being
		 * applied to the body.
		 */
		void AddForceAtWorldPoint(const AeroVec3& force, const AeroVec3& point);

		/**
		 * \brief Adds the given force to the given point on the rigid body.
		 * The force is still expected to be in world coordinates.
		 * \param force The force to apply given in WORLD coordinates.
		 * \param point The point on the rigid body to apply the force to
		 * given in BODY coordinates. This function may result in a torque being
		 * applied to the body.
		 */
		void AddForceAtBodyPoint(const AeroVec3& force, const AeroVec3& point);

		/**
		 * \brief Can be used to directly move the rigid body to a specified position.
		 * \param p The position to move the rigid body to.
		 */
		void SetPosition(const AeroVec3& p);

		/**
		 * \brief Cam ne used to directly move the rigid body to the specified position.
		 * \param x The x coordinate of the new position.
		 * \param y The y coordinate of the new position.
		 * \param z The z coordinate of the new position.
		 */
		void SetPosition(const real x, const real y, const real z);

		/**
		 * \brief Converts a point defined in world space to a point defined in this rigid bodies local space.
		 * \param point The point in world space to get in the body space of this rigid body.
		 * \return Returns a point with respect to the rigid bodies center of mass.
		 */
		AeroVec3 GetPointInLocalSpace(const AeroVec3& point) const;

		/**
		* \brief Converts a point defined in this bodies local space to a point defined in world space.
		* \param point The point in local space with respect to this body.
		* \return Returns a point defined in world space.
		*/
		AeroVec3 GetPointInWorldSpace(const AeroVec3& point) const;

	};
}


#endif
