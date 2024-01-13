#include "Particle.h"
#include <iostream>
using namespace Aerolite;

Particle::Particle(real x, real y, real mass) : position(x, y), mass(mass)
{
	radius = 4.0;
	velocity = Vec2(0.0, 0.0);
	acceleration = Vec2(0.0, 0.0);
	netForces = Vec2(0.0, 0.0);
	if (mass != 0.0)
		invMass = 1.0 / mass;
	else
		invMass = 0.0;

	std::cout << "Particle constructor called" << std::endl;
}

// Copy constructor
Particle::Particle(const Particle& other)
    : radius(other.radius),
    position(other.position),
    velocity(other.velocity),
    acceleration(other.acceleration),
    netForces(other.netForces),
    mass(other.mass),
    invMass(other.invMass)
{
    // No need to copy Vec2 objects explicitly as their copy constructors will be called.
}

// Move constructor
Particle::Particle(Particle&& other) noexcept
    : radius(std::exchange(other.radius, 0)),
    position(std::move(other.position)),
    velocity(std::move(other.velocity)),
    acceleration(std::move(other.acceleration)),
    netForces(std::move(other.netForces)),
    mass(std::exchange(other.mass, 0)),
    invMass(std::exchange(other.invMass, 0))
{
    // No need to move Vec2 objects explicitly as their move constructors will be called.
}

// Copy assignment operator
Particle& Particle::operator=(const Particle& other) {
    if (this != &other) {
        radius = other.radius;
        position = other.position;
        velocity = other.velocity;
        acceleration = other.acceleration;
        netForces = other.netForces;
        mass = other.mass;
        invMass = other.invMass;
        // No need to copy Vec2 objects explicitly as their copy assignment operators will be called.
    }
    return *this;
}

// Move assignment operator
Particle& Particle::operator=(Particle&& other) noexcept {
    if (this != &other) {
        radius = std::exchange(other.radius, 0);
        position = std::move(other.position);
        velocity = std::move(other.velocity);
        acceleration = std::move(other.acceleration);
        netForces = std::move(other.netForces);
        mass = std::exchange(other.mass, 0);
        invMass = std::exchange(other.invMass, 0);
        // No need to move Vec2 objects explicitly as their move assignment operators will be called.
    }
    return *this;
}

Particle::~Particle()
{
	std::cout << "Particle destructor called" << std::endl;
}

void Particle::ApplyForce(const Vec2& force)
{
	netForces += force;
}

void Particle::ClearForces()
{
	netForces = Vec2(0.0f, 0.0f);
}

bool Aerolite::Particle::HasFiniteMass(void)
{
	return invMass != 0.0f;
}

void Particle::Integrate(const real dt)
{
	// Find the acceleration based on forces being applied.
	acceleration = netForces * invMass;

	// Integrate acceleration to find velocity;
	velocity += acceleration * dt;

	// Integrate velocity to find position.
	position += velocity * dt + (acceleration * dt * dt) / 2.0f;

	ClearForces();
}