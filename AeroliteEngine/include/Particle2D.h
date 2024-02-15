#ifndef PARTICLE_H
#define PARTICLE_H

#include "AeroVec2.h"
#include "Precision.h"

namespace Aerolite {
    // The Particle2D struct represents a simple physical particle in a 2D space.
    // It is designed to simulate basic physics properties like position, linear_velocity, 
    // and linear_acceleration, as well as handling the integration of these properties over time.
    struct Particle2D {
        int radius;          // The radius of the particle. Useful for rendering or collision detection.

        AeroVec2 position;       // The current position of the particle in 2D space.
        AeroVec2 velocity;       // The current linear_velocity of the particle. Determines how fast and in what direction it moves.
        AeroVec2 acceleration;   // The current linear_acceleration of the particle. Influences the change in linear_velocity.
        AeroVec2 netForces;      // The current sum of all forces acting on this particle. Influences the linear_acceleration of the particle.
        // Useful for applying multiple forces and calculating the resulting linear_acceleration.

        real mass;          // The mass of the particle. This can be used to influence the motion in a physical simulation (e.g., under gravity).
        real invMass;       // The inverse mass of the particle. Useful for optimizing calculations involving mass,
        // such as force application. It is particularly handy in physics engines where 
        // division by mass is common, as multiplying by the inverse mass is more efficient.

        // Constructor for the Particle2D. Initializes the particle with a position (x, y), and mass.
        // The linear_velocity and linear_acceleration are initialized to zero by default.
        Particle2D(real x, real y, real mass);

        ~Particle2D() = default;

        // Copy constructor
        Particle2D(const Particle2D& other) = default;

        // Move constructor
        Particle2D(Particle2D&& other) noexcept = default;

        // Copy assignment operator
        Particle2D& operator=(const Particle2D& other);

        // Move assignment operator
        Particle2D& operator=(Particle2D&& other) noexcept = default;

        /// <summary>
        /// Function used to apply forces to this particle to change its linear_acceleration.
        /// </summary>
        /// <param name="force">The force to be applied to the particle.</param>
        void ApplyForce(const AeroVec2& force);

        // Clears all forces that were applied to the particle for the current frame.
        void ClearForces();

        // Check to see if the mass of the object is extremely large. It does this by checking if
        // the inverse mass is zero.
        bool HasFiniteMass(void) const;

        // Integrate function updates the physics state of the particle. This includes updating 
        // the position based on linear_velocity and linear_acceleration over a given time step (dt).
        // This is often called in the update loop of a simulation or game to simulate motion.
        void Integrate(const real dt);
    };
}


#endif // PARTICLE_H
