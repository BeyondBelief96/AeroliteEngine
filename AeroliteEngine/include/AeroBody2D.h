#ifndef BODY2D_H
#define BODY2D_H

#include "AeroVec2.h"
#include "Shape.h"
#include "Precision.h"
#include "AABB2D.h"

namespace Aerolite {

    /**
     * \brief Represents a 2-Dimensional physical body. This is the basic simulation
     * object for the 2D portion of the physics engine.
     */
    struct AeroBody2D
    {
        /**
         * \brief Position of the body in 2D space.
         */
        AeroVec2 position;

        /**
         * \brief Velocity of the body.
         */
        AeroVec2 velocity;

        /**
         * \brief Acceleration of the body.
         */
        AeroVec2 acceleration;

        /**
         * \brief Rotation angle of the body.
         */
        real rotation;

        /**
         * \brief Angular velocity of the body.
         */
        real angular_velocity;

        /**
         * \brief Angular acceleration of the body.
         */
        real angular_acceleration;

        /**
         * \brief Sum of forces currently applied to the body.
         */
        AeroVec2 sum_forces;

        /**
         * \brief Sum of all torques currently being applied to the body.
         */
        real sum_torque;

        /**
         * \brief Mass of the body. Initialized to 0.0.
         */
        real mass = 0.0f;

        /**
         * \brief Inverse mass of the body. Useful for physics calculations.
         */
        real inv_mass = 0.0f;

        /**
         * \brief Moment of inertia of the rigid body. This is based on the body's shape.
         */
        real I;

        /**
         * \brief Inverse of the moment of inertia of the rigid body. More efficient and has better numerical stability.
         */
        real invI;

        /**
         * \brief Coefficient of restitution for the body.
         */
        real restitution;

        /**
         * \brief Coefficient of friction of the body. Used in tangent impulse calculations.
         */
        real friction;

        /**
         * \brief Raw pointer to a Shape object. Initialized to nullptr.
         */
        Shape* shape = nullptr;

        /**
         * \brief Constructor for AeroBody2D.
         */
        AeroBody2D(Shape* shape, const real x, const real y, const real mass);

        /**
         * \brief Destructor for AeroBody2D.
         */
        ~AeroBody2D();

        /**
         * \brief Returns an AABB surrounding the rigid body for broad phase collision detection.
         */
        AABB2D GetAABB(void) const;

        /**
         * \brief Performs integration of the forces/torques and linear/angular accelerations to find the new velocities of the current simulation frame.
         */
        void IntegrateForces(real dt);

        /**
         * \brief Performs integration of the linear and angular velocities to find the new position/rotation of the current simulation frame.
         */
        void IntegrateVelocities(real dt);

        /**
         * \brief Determines if the AeroBody2D has an infinite mass.
         * \return Returns true if mass is zero, false otherwise.
         */
        bool IsStatic(void) const;

        /**
         * \brief Method to add a force vector to the body.
         */
        void AddForce(const AeroVec2& force);

        /**
         * \brief Method to add a torque to the body.
         */
        void AddTorque(const real torque);

        /**
         * \brief Applies a linear impulse to the body at the center of mass.
         */
        void ApplyImpulseLinear(const AeroVec2& j);

        /**
         * \brief Applies an angular impulse to the body at the center of mass.
         */
        void ApplyImpulseAngular(const real j);

        /**
         * \brief Applies an impulse j to the body at the point r generated from a collision.
         */
        void ApplyImpulseAtPoint(const AeroVec2& j, const AeroVec2& r);

        /**
         * \brief Method to clear all forces acting on the body.
         */
        void ClearForces(void);

        /**
         * \brief Method to clear all torques acting on the body.
         */
        void ClearTorque(void);

        /**
         * \brief Converts a given point defined in local space of this body to world space coordinates with respect to this body.
         */
        AeroVec2 LocalSpaceToWorldSpace(const AeroVec2& point) const;

        /**
         * \brief Converts a given point defined in world space of this body to local space coordinates with respect to this body.
         */
        AeroVec2 WorldSpaceToLocalSpace(const AeroVec2& point) const;
    };
}

#endif

