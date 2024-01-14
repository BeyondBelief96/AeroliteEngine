// Include guard to prevent multiple inclusions of this header file.
#ifndef BODY2D_H
#define BODY2D_H

#include <memory>
#include "Vec2.h" 
#include "Shape.h"
#include "Precision.h"
#include "AABB2D.h"

// Using the Aerolite namespace throughout this file.
// Note: Using a whole namespace in a header can sometimes lead to conflicts in large projects.
using Vec2 = Aerolite::Vec2;
using Shape = Aerolite::Shape;
using real = Aerolite::real;

// Declare everything in this header in the Aerolite namespace.
namespace Aerolite {

    // Define a struct Body2D representing a 2D physical body.
    struct Body2D 
    {
        bool isColliding = false;
        Vec2 position; // Position of the body in 2D space.
        Vec2 velocity; // Velocity of the body.
        Vec2 acceleration; // Acceleration of the body.

        real rotation; // Rotation angle of the body.
        real angularVelocity; // Angular velocity of the body.
        real angularAcceleration; // Angular acceleration of the body.
        Vec2 sumForces; // Sum of forces currently applied to the body.
        real sumTorque; // Sum of all torques currently being applied to the body.

        real mass = 0.0f; // Mass of the body. Initialized to 0.0.
        real invMass = 0.0f; // Inverse mass of the body. Useful for physics calculations.
        real I; // Moment of inertia of the rigid body. This is based on the body's shape.
        real invI; // Inverse of the moment of inertia of the rigid body. Inverse moment of inertia is 
                   // more efficient and has better numerical stability.

        real restitution; // Coefficient of restitution for the body.
        real friction;    // Coefficient of friction of the body. Used in tangent impulse calculations.
        
        Shape* shape = nullptr; // Raw pointer to a Shape object. Initialized to nullptr.


        // Constructor for Body2D.
        Body2D(Shape* shape, const real x, const real y, const real mass);

        // Destructor for Body2D.
        ~Body2D();

        /// <summary>
        /// Returns an AABB surrounding the rigid body for broad phase collision detection.
        /// </summary>
        /// <returns>The AABB for collision detection.</returns>
        AABB2D GetAABB(void);

        /// <summary>
        /// Updates the body by preforming linear and angular integration of the accelerations to derive the new positions
        /// for the current time step of the simulation.
        /// </summary>
        /// <param name="dt">The current time step of the simulation frame.</param>
        void Update(const real dt); 

        /// @brief Determines if the Body2D has an infinite mass.
        /// @return Returns true if mass is zero, false otherwise.
        bool IsStatic(void) const;
        
        // Method to add a force vector to the body.
        void AddForce(const Vec2& force);

        // Method to add a torque to the body.
        void AddTorque(const real torque);
           
        /// <summary>
        /// Applies an impulse to the body at the center of mass generated from a collision.
        /// </summary>
        /// <param name="j">The impulse vector to apply.</param>
        void ApplyImpulse(const Vec2& j);

        /// <summary>
        /// Applies an impulse j to the body at the point r generated from a collision.
        /// </summary>
        /// <param name="j">The impulse vector to apply.</param>
        /// <param name="r">The point on the body to apply the impulse.</param>
        void ApplyImpulse(const Vec2& j, const Vec2& r);

        // Method to clear all forces acting on the body.
        void ClearForces(void);

        // Method to clear all torques acting on the body.
        void ClearTorque(void);

        // Method to integrate the body's linear position and velocity over time (dt).
        void IntegrateLinear(const real dt);

        // Method to integrate the body's rotation and angular velocity over time (dt).
        void IntegrateAngular(const real dt);

        /// <summary>
        /// Converts a given point defined in local space of this body to world space coordinates
        /// with respect to this body.
        /// </summary>
        /// <param name="point">The point in local space with respect to the body.</param>
        /// <returns>A point in world space with respect to the body.</returns>
        Aerolite::Vec2 LocalSpaceToWorldSpace(const Vec2& point);

        /// <summary>
        /// Converts a given point defined in world space of this body to local space coordinates
        /// with respect to this body.
        /// </summary>
        /// <param name="point">The point in world space with respect to the body.</param>
        /// <returns>A point in local space with respect to the body.</returns>
        Aerolite::Vec2 WorldSpaceToLocalSpace(const Vec2& point);
    };
}

#endif
