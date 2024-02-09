#ifndef BODY2D_H
#define BODY2D_H

#include "AeroVec2.h"
#include "Shape.h"
#include "Precision.h"
#include "AeroAABB2D.h"

namespace Aerolite {

    /**
     * \brief Represents a 2-Dimensional physical body. This is the basic simulation
     * object for the 2D portion of the physics engine.
     */
    class AeroBody2D
    {
    public:
        aero_uint16 id; ///< Unique identifier for the body.
        AeroVec2 position; ///< Position of the body in world space coordinates.
        AeroVec2 linear_velocity; ///< Linear linear_velocity of the body.
        AeroVec2 linear_acceleration; ///< Linear linear_acceleration of the body.
        real rotation; ///< Current rotation angle (in radians) of the body around the Z-axis.
        real angular_velocity; ///< Angular linear_velocity of the body (rad/s).
        real angular_acceleration; ///< Angular linear_acceleration of the body (rad/s^2).
        real linear_damping; ///< Coefficient for linear damping, reducing linear linear_velocity over time.
        real angular_damping; ///< Coefficient for angular damping, reducing angular linear_velocity over time.
        AeroVec2 sum_forces; ///< Accumulated sum of forces applied to the body.
        real sum_torque; ///< Accumulated sum of torques applied to the body.
        real mass; ///< Mass of the body. A mass of 0 indicates a static body.
        real inv_mass; ///< Inverse mass of the body for efficient computation. Zero for static bodies.
        real inertia; ///< Moment of inertia of the body, defining resistance to rotational linear_acceleration.
        real inv_inertia; ///< Inverse of the moment of inertia for efficient computation.
        real restitution; ///< Coefficient of restitution (bounciness) of the body.
        real friction; ///< Coefficient of friction affecting tangential collision response.
        Shape* shape; ///< Shape of the body, defining its geometric representation.
        bool is_sleeping; ///< Flag indicating if the body is currently in a sleeping state to optimize simulation.
        unsigned int sleep_timer; ///< Timer for tracking how long the body has been inactive.

    public:
        /**
         * @brief Construct a new AeroBody2D object with specified shape, position, and mass.
         *
         * @param shape Pointer to the shape representing the body's geometry.
         * @param x Initial x-coordinate of the body's position.
         * @param y Initial y-coordinate of the body's position.
         * @param mass Mass of the body, where a value of 0 indicates a static (immovable) body.
         */
        AeroBody2D(Shape* shape, const real x, const real y, const real mass);

        /**
         * @brief Destroy the AeroBody2D object, performing necessary cleanup.
         */
        ~AeroBody2D();

        /**
         * @brief Calculates and returns the Axis-Aligned Bounding Box (AABB) of the body.
         *
         * @return AeroAABB The bounding box encapsulating the body.
         */
        AeroAABB2D GetAABB() const;

        /**
         * @brief Integrates the body's forces to update its linear and angular accelerations.
         *
         * @param dt Time step for the integration.
         */
        void IntegrateForces(real dt);

        /**
         * @brief Integrates the body's velocities to update its position and rotation.
         *
         * @param dt Time step for the integration.
         */
        void IntegrateVelocities(real dt);

        /**
         * @brief Checks if the body is static, meaning it has an infinite mass and does not respond to forces.
         *
         * @return true if the body is static, false otherwise.
         */
        bool IsStatic() const;

        /**
         * @brief Adds a force to the body's force accumulator.
         *
         * @param force The force vector to add.
         */
        void AddForce(const AeroVec2& force);

        /**
         * @brief Adds a torque to the body's torque accumulator.
         *
         * @param torque The torque value to add.
         */
        void AddTorque(const real torque);

        /**
         * @brief Applies a linear impulse to the body, affecting its linear velocity.
         *
         * @param j The impulse vector to apply.
         */
        void ApplyImpulseLinear(const AeroVec2& j);

        /**
         * @brief Applies an angular impulse to the body, affecting its angular velocity.
         *
         * @param j The angular impulse value to apply.
         */
        void ApplyImpulseAngular(const real j);

        /**
         * @brief Applies an impulse at a specific point on the body, affecting both linear and angular velocities.
         *
         * @param j The impulse vector to apply.
         * @param r The point (relative to the body's center of mass) where the impulse is applied.
         */
        void ApplyImpulseAtPoint(const AeroVec2& j, const AeroVec2& r);

        /**
         * @brief Puts this body to sleep. All forces, velocities, accelerations are set to zero.
         */
        void Sleep();

        /**
         * @brief Awakens the body.
         */
        void Awake();

        /**
         * @brief Clears all accumulated forces on the body.
         */
        void ClearForces();

        /**
         * @brief Clears all accumulated torque on the body.
         */
        void ClearTorque();

        /**
         * @brief Converts a point from the body's local space to world space coordinates.
         *
         * @param point The point in local space to convert.
         * @return AeroVec2 The point in world space coordinates.
         */
        AeroVec2 LocalSpaceToWorldSpace(const AeroVec2& point) const;

        /**
         * @brief Converts a point from world space to the body's local space coordinates.
         *
         * @param point The point in world space to convert.
         * @return AeroVec2 The point in local space coordinates.
         */
        AeroVec2 WorldSpaceToLocalSpace(const AeroVec2& point) const;
    };
}

#endif

