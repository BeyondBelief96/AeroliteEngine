#ifndef PFGEN_H
#define PFGEN_H

#include <vector>
#include <memory>
#include "Particle.h"
#include "Precision.h"
#include "Vec2.h"

namespace Aerolite {

    // ParticleForceGenerator is an abstract base class representing a force generator
    // that can act on particles. It defines a pure virtual function UpdateForce which 
    // must be implemented by derived classes to update the force on a particle.
    class ParticleForceGenerator {
    public:
        // Pure virtual function that applies a force to a particle over a time interval.
        // Must be implemented by derived force generators.
        virtual void UpdateForce(std::shared_ptr<Particle> particle, real dt) = 0;
    };

    // ParticleForceRegistry maintains a list of particle-force generator associations.
    // It allows the application of forces to particles and handles the update of these forces.
    class ParticleForceRegistry {
    protected:
        // Nested struct to associate a Particle with a ForceGenerator.
        struct ParticleForceRegistration {
            std::shared_ptr<Particle> particle;             // Pointer to the particle
            std::shared_ptr<ParticleForceGenerator> fg;     // Pointer to the force generator affecting the particle
        };

        // Typedef for a vector of ParticleForceRegistration, representing the registry.
        typedef std::vector<ParticleForceRegistration> Registry;
        Registry registrations;            // Vector storing all particle-force generator associations

    public:
        // Adds a particle and its associated force generator to the registry.
        void Add(std::shared_ptr<Particle> particle, std::shared_ptr<ParticleForceGenerator> fg);

        // Clears all registrations from the registry.
        void Clear(void);

        // Updates the forces on all registered particles.
        void UpdateForces(const real dt);
    };

    // ParticleGravity: A force generator that simulates the effect of a uniform gravitational field on particles.
    // This class is specifically designed for scenarios where particles are affected by a constant gravity,
    // such as the gravitational pull of the Earth. It differs from ParticleGAttraction, which
    // is used to simulate the gravitational attraction between two particles based on their masses and distance.
    // ParticleGravity inherits from ParticleForceGenerator and overrides the UpdateForce method to apply a uniform
    // gravitational force to a single particle.
    class ParticleGravity : public ParticleForceGenerator {
    public:
        // Constructor for ParticleGravity.
        // gravity: A Vec2 representing the gravitational force to be applied (typically pointing downwards).
        ParticleGravity(const Vec2& gravity);

        // Overrides the UpdateForce method to apply gravitational force to a particle.
        // particle: The particle to which the force will be applied.
        // dt: The duration over which to apply the force (usually a time step in the simulation).
        virtual void UpdateForce(std::shared_ptr<Particle> particle, real dt) override;

    private:
        Vec2 gravity;  // The gravitational force vector. This is usually a constant value, like gravity on Earth.
    };

    // ParticleDrag is a force generator that simulates drag forces acting on a particle.
    // Drag forces typically oppose the direction of velocity and increase with speed.
    // It inherits from ParticleForceGenerator and implements the UpdateForce method.
    class ParticleDrag : public ParticleForceGenerator {
    public:
        // Constructor for ParticleDrag.
        // linVelDragCoef: Linear drag coefficient (k1) - affects drag linearly with velocity.
        // velSqDragCoeff: Quadratic drag coefficient (k2) - af with the square of velfects dragocity.
        ParticleDrag(real linVelDragCoef, real velSqDragCoeff);

        // Overrides the UpdateForce method to apply drag force to a particle.
        // particle: The particle to which the force will be applied.
        // dt: The duration over which to apply the force (usually a time step in the simulation).
        virtual void UpdateForce(std::shared_ptr<Particle> particle, real dt) override;

    private:
        real k1;  // The linear drag coefficient. Controls how much the velocity directly affects drag.
        real k2;  // The quadratic drag coefficient. Controls how much the velocity squared affects drag.
    };

    // ParticleFriction class defines a friction force generator for particles.
    // It inherits from ParticleForceGenerator, which is an abstract class for particle force generators.
    class ParticleFriction : public ParticleForceGenerator {
    public:
        // Constructor that initializes the ParticleFriction with a friction coefficient.
        // frictionCoeff: The coefficient of friction. This value determines how much the force of friction
        //                affects the particle. A higher value means stronger friction.
        ParticleFriction(real frictionCoeff);

        // Override of the pure virtual function UpdateForce from ParticleForceGenerator.
        // This function applies a frictional force to the particle.
        // particle: The shared pointer to the Particle object on which the force will be applied.
        //           Using a shared pointer ensures proper memory management and shared ownership,
        //           which can be useful when particles are accessed from multiple places.
        // dt: The time step over which to apply the force. This is typically the duration of a single
        //     frame in a simulation or game loop.
        virtual void UpdateForce(std::shared_ptr<Particle> particle, real dt) override;

    private:
        real frictionCoeff; // Coefficient of friction. This member variable stores the value set by the constructor.
        // The frictional force is generally proportional to the coefficient of friction and the normal force.
        // In a particle system, it's commonly used to simulate resistance against the motion of the particle.
    };

    // The ParticleGAttraction class is a specialized force generator that simulates
    // the gravitational force between two particles. It inherits from ParticleForceGenerator, 
    // providing a specific implementation for gravitational attraction.
    class ParticleGAttraction : public ParticleForceGenerator {
    public:
        // Constructor for the ParticleGravitationalForceGenerator class.
        // This constructor initializes a gravitational force generator with two particles and a gravitational constant.
        // The gravitational force is calculated based on these two particles and applied to particleA by particleB.
        //
        // Parameters:
        // particleA: A shared pointer to the first Particle object. This is the particle that will have the gravitational 
        //            force applied to it by particleB. The shared pointer ensures proper memory management and ownership sharing.
        //
        // particleB: A shared pointer to the second Particle object. This is the particle that exerts the gravitational 
        //            force on particleA. Similar to particleA, the shared pointer manages memory and ownership.
        //
        // gravitationalConstant: A real representing the gravitational constant to be used in the force calculation. 
        //                         This constant influences the strength of the gravitational force between the two particles.
        //
        // minDistance and maxDistance: These parameters are used to clamp the distance between particleA and particleB
        //                              within a specified range before calculating the gravitational force. The minDistance
        //                              is used to avoid extremely high forces at very small distances (avoiding singularity),
        //                              while the maxDistance sets an upper limit beyond which the gravitational attraction
        //                              becomes negligible. This approach provides a realistic and computationally efficient
        //                              simulation of gravitational forces, as it avoids the calculation of forces for
        //                              extremely close or distant particles.
        //
        ParticleGAttraction(std::shared_ptr<Particle> particleA,
            std::shared_ptr<Particle> particleB,
            real gravitationalConstant, double minDistance, double maxDistance);

        // Overrides the UpdateForce method from ParticleForceGenerator. 
        // This method calculates and applies the gravitational force exerted by particleB on particleA.
        // particle: The particle to which this particular force generator instance is being applied. 
        //           In the context of this class, it should be the same as particleA.
        // dt: The time step over which to apply the force. This is typically the duration of a single
        //     frame in a simulation or game loop.
        virtual void UpdateForce(std::shared_ptr<Particle> particle, real dt) override;

    private:
        // Shared pointers to the particles involved in the gravitational interaction.
        // Using shared pointers ensures proper memory management and shared ownership, 
        // which is useful when particles are accessed from multiple places.
        std::shared_ptr<Particle> particleA;  // The particle that receives the gravitational force.
        std::shared_ptr<Particle> particleB;  // The particle that exerts the gravitational force.

        real gravConstant;  // The gravitational constant used in the force calculation. 
        // This constant is key in determining the magnitude of the force.

        double minDistance;   // The minimum distance at which the gravitational force is effective.
        // This is used to avoid extremely high forces at very small distances (singularity problem).
        // It can also represent a physical limit, like the surface distance where the particles would collide.

        double maxDistance;   // The maximum distance beyond which the gravitational force is not considered.
        // This can be used to optimize calculations by ignoring negligible forces at large distances.
        // It helps in managing computational efficiency, especially in simulations with many particles.
    };

    // ParticleSpring: A class derived from ParticleForceGenerator. 
    // It models a spring force acting on a particle. This force depends on the distance 
    // between the particle and a fixed anchor point in space.
    class ParticleSpringAnchored : public ParticleForceGenerator
    {
    public:
        // Constructor for the ParticleSpring class.
        // Initializes a new ParticleSpring with a target particle, an anchor point, 
        // the rest length of the spring, and the spring constant.
        // 
        // @param anchor: The Vec2 object representing the fixed anchor point in space.
        // @param restLength: The natural length of the spring at which no force is exerted.
        // @param springConstant: The stiffness of the spring, determining how much force is applied 
        //                        for a given extension or compression.
        ParticleSpringAnchored(const Vec2 anchor, real restLength, real springConstant);

        // UpdateForce: Override the virtual function from ParticleForceGenerator.
        // This method calculates and applies the spring force to the attached particle.
        // 
        // @param particle: The shared pointer to the Particle object on which the force is to be applied.
        // @param dt: The time interval over which the force acts.
        virtual void UpdateForce(std::shared_ptr<Particle> particle, real dt) override;

    private:
        Vec2 anchor;     // The fixed anchor point in space to which the spring is attached.
        real restLength; // The natural length of the spring (length at rest).
        real k;         // The spring constant, representing the stiffness of the spring.
    };

    // ParticleSpring: A class derived from ParticleForceGenerator that models a spring force between two particles.
// This class represents a spring connecting two particles, applying forces based on the spring's properties.
class ParticleSpring : public ParticleForceGenerator 
{
private:
    // particleA: A shared pointer to the first Particle object connected by the spring.
    std::shared_ptr<Particle> particleA;

    // particleB: A shared pointer to the second Particle object connected by the spring.
    std::shared_ptr<Particle> particleB;

    // l0: The rest length of the spring. This is the length at which the spring neither exerts compression nor tension.
    real l0;

    // k: The spring constant, representing the stiffness of the spring. Higher values make the spring stiffer.
    real k;

public:
    // Constructor for the ParticleSpring class.
    // Initializes a new ParticleSpring connecting two particles with specified rest length and spring constant.
    // 
    // @param particleA: A shared pointer to the first Particle object to be connected by the spring.
    // @param particleB: A shared pointer to the second Particle object to be connected by the spring.
    // @param restLength: The natural (rest) length of the spring.
    // @param springConstant: The stiffness of the spring.
    ParticleSpring(std::shared_ptr<Particle> particleA, std::shared_ptr<Particle> particleB, const real restLength, const real springConstant);

    // UpdateForce: Overrides the virtual function from ParticleForceGenerator.
    // This method calculates and applies the spring force to each of the two connected particles.
    // The force applied is based on the extension or compression of the spring relative to its rest length.
    // 
    // @param particle: The shared pointer to the Particle object on which the force is to be applied.
    // @param dt: The time interval over which the force acts.
    virtual void UpdateForce(std::shared_ptr<Particle> particle, real dt) override;
};


}

#endif // PFGEN_H
