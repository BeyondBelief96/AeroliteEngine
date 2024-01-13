#ifndef PARTICLE_H
#define PARTICLE_H

#include "Vec2.h"
#include "Precision.h"

using namespace Aerolite;

namespace Aerolite {
    // The Particle struct represents a simple physical particle in a 2D space.
    // It is designed to simulate basic physics properties like position, velocity, 
    // and acceleration, as well as handling the integration of these properties over time.
    struct Particle {
        int radius;          // The radius of the particle. Useful for rendering or collision detection.

        Vec2 position;       // The current position of the particle in 2D space.
        Vec2 velocity;       // The current velocity of the particle. Determines how fast and in what direction it moves.
        Vec2 acceleration;   // The current acceleration of the particle. Influences the change in velocity.
        Vec2 netForces;      // The current sum of all forces acting on this particle. Influences the acceleration of the particle.
        // Useful for applying multiple forces and calculating the resulting acceleration.

        real mass;          // The mass of the particle. This can be used to influence the motion in a physical simulation (e.g., under gravity).
        real invMass;       // The inverse mass of the particle. Useful for optimizing calculations involving mass,
        // such as force application. It is particularly handy in physics engines where 
        // division by mass is common, as multiplying by the inverse mass is more efficient.

        // Constructor for the Particle. Initializes the particle with a position (x, y), and mass.
        // The velocity and acceleration are initialized to zero by default.
        Particle(real x, real y, real mass);

        // Copy constructor
        Particle(const Particle& other);

        // Move constructor
        Particle(Particle&& other) noexcept;

        // Copy assignment operator
        Particle& operator=(const Particle& other);

        // Move assignment operator
        Particle& operator=(Particle&& other) noexcept;

        // Destructor for the Particle. Currently, it does not need to perform any specific actions
        // as the struct does not manage any resources directly.
        ~Particle();

        /// <summary>
        /// Function used to apply forces to this particle to change its acceleration.
        /// </summary>
        /// <param name="force">The force to be applied to the particle.</param>
        void ApplyForce(const Vec2& force);

        // Clears all forces that were applied to the particle for the current frame.
        void ClearForces();

        // Check to see if the mass of the object is extremely large. It does this by checking if
        // the inverse mass is zero.
        bool HasFiniteMass(void);

        // Integrate function updates the physics state of the particle. This includes updating 
        // the position based on velocity and acceleration over a given time step (dt).
        // This is often called in the update loop of a simulation or game to simulate motion.
        void Integrate(const real dt);
    };
}


#endif // PARTICLE_H
