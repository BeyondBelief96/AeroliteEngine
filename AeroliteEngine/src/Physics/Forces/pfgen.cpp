#include <algorithm>
#include "pfgen.h"

using namespace Aerolite;

// Add a particle and its associated force generator to the registry.
void Aerolite::ParticleForceRegistry::Add(std::shared_ptr<Particle> particle, std::shared_ptr<ParticleForceGenerator> fg) {
    // Emplace_back efficiently constructs a ParticleForceRegistration in place,
    // thereby avoiding unnecessary copy or move operations.
    registrations.emplace_back(ParticleForceRegistration{ particle, fg });
}

// Clear all force generator registrations from the registry.
void Aerolite::ParticleForceRegistry::Clear() {
    // Clear the registrations vector. This action destroys all ParticleForceRegistration objects
    // and sets the size of the vector to 0.
    registrations.clear();
}

// Update forces on all particles registered with force generators.
void ParticleForceRegistry::UpdateForces(const float dt) {
    // Iterate through each registration in the registry.
    for (Registry::iterator i = registrations.begin(); i != registrations.end(); i++) {
        // Invoke UpdateForce on the force generator, passing the associated particle and the time step dt.
        // This updates the force on each particle based on the specific logic of the force generator.
        i->fg->UpdateForce(i->particle, dt);
    }

    // Clear the registry after updating forces to reset and prepare for the next update cycle.
    Clear();
}

// Constructor to initialize the gravitational force.
Aerolite::ParticleGravity::ParticleGravity(const Vec2& gravity) {
    // Set the internal gravity vector to the specified gravity value.
    this->gravity = gravity;
}

// Apply gravitational force to a particle.
void Aerolite::ParticleGravity::UpdateForce(std::shared_ptr<Particle> particle, float dt) {
    // Check if the particle has finite mass. Skip force application if mass is infinite (immovable object).
    if (!particle->HasFiniteMass()) return;

    // Apply a gravitational force that is proportional to the particle's mass.
    // The force is in the direction of the gravity vector.
    particle->ApplyForce(gravity * particle->mass);
}

// Constructor to initialize the drag coefficients.
Aerolite::ParticleDrag::ParticleDrag(float linVelDragCoef, float velSqDragCoeff) {
    // Set the linear and quadratic drag coefficients.
    k1 = linVelDragCoef;
    k2 = velSqDragCoeff;
}

// Apply drag force to a particle.
void Aerolite::ParticleDrag::UpdateForce(std::shared_ptr<Particle> particle, float dt) {
    // Only apply drag if the particle is moving (velocity magnitude is greater than zero).
    if (particle->velocity.MagnitudeSquared() > 0) {
        // Calculate the drag force direction, opposite to the particle's velocity.
        Vec2 dragForce = particle->velocity.UnitVector() * -1.0;

        // Compute the magnitude of drag using the drag equation, which includes linear and quadratic components.
        float dragMagnitude = particle->velocity.Magnitude() * k1 + particle->velocity.MagnitudeSquared() * k2;

        // Scale the drag force by its magnitude.
        dragForce *= dragMagnitude;

        // Apply the drag force to the particle.
        particle->ApplyForce(dragForce);
    }
}

// Constructor to initialize the friction coefficient.
Aerolite::ParticleFriction::ParticleFriction(float frictionCoeff) {
    // Set the internal friction coefficient to the specified value.
    this->frictionCoeff = frictionCoeff;
}

// Apply friction force to a particle.
void Aerolite::ParticleFriction::UpdateForce(std::shared_ptr<Particle> particle, float dt) {
    // Calculate the direction of friction force, which is opposite to the particle's velocity direction.
    Vec2 frictionDirection = particle->velocity.UnitVector() * -1.0;

    // The magnitude of the friction force is determined by the friction coefficient.
    float frictionMagnitude = frictionCoeff;

    // Construct the friction force vector by combining its direction and magnitude.
    Vec2 frictionForce = frictionDirection * frictionMagnitude;

    // Apply the friction force to the particle.
    particle->ApplyForce(frictionForce);
}

// Constructor to initialize particles involved in gravitational attraction and the gravitational constant.
Aerolite::ParticleGAttraction::ParticleGAttraction(std::shared_ptr<Particle> particleA, std::shared_ptr<Particle> particleB,
    float gravitationalConstant, float minDistance, float maxDistance) {
    // Set the internal members for particles and gravitational constant.
    this->particleA = particleA;
    this->particleB = particleB;
    this->gravConstant = gravitationalConstant;

    // Set the minimum and maximum effective distances for gravitational attraction.
    this->minDistance = minDistance;
    this->maxDistance = maxDistance;
}

// Apply gravitational attraction force between two particles.
void Aerolite::ParticleGAttraction::UpdateForce(std::shared_ptr<Particle> particle, float dt) {
    // Calculate the displacement vector between particleB and particleA.
    Vec2 d = (particleB->position - particleA->position);

    // Compute the squared distance between the particles.
    float distanceSquared = d.MagnitudeSquared();

    // Clamp the distance to be within the specified min and max range to avoid extreme forces.
    distanceSquared = std::clamp(distanceSquared, minDistance, maxDistance);

    // Calculate the magnitude of gravitational attraction using Newton's law of universal gravitation.
    float attractionMagnitude = gravConstant * (particleA->mass * particleB->mass) / distanceSquared;

    // Construct the gravitational force vector, directed towards particleB.
    Vec2 attractionForce = d.UnitVector() * attractionMagnitude;

    // Apply the gravitational force to the particle.
    particle->ApplyForce(attractionForce);
}

Aerolite::ParticleSpringAnchored::ParticleSpringAnchored(const Vec2 anchor, float restLength, float springConstant)
{
    this->anchor = anchor;
    this->restLength = restLength;
    this->k = springConstant;
}

void Aerolite::ParticleSpringAnchored::UpdateForce(std::shared_ptr<Particle> particle, float dt)
{
    // Calculate the distance between the anchor and the particle.
    Vec2 d = particle->position - anchor;

    // Find the spring displacement considering the rest length;
    float displacement = d.Magnitude() - restLength;

    // Calculate the direction & magnitude of the spring force.
    Vec2 springDirection = d.UnitVector();
    float springMagnitude = -k * displacement;

    // Calculate the final resulting spring force vector.
    Vec2 springForce = springDirection * springMagnitude;
    particle->ApplyForce(springForce);
}
