#include "Particle2D.h"
#include <iostream>
using namespace Aerolite;

Particle2D::Particle2D(real x, real y, real mass) : position(x, y), mass(mass)
{
	radius = 4.0;
	velocity = Vec2(0.0, 0.0);
	acceleration = Vec2(0.0, 0.0);
	netForces = Vec2(0.0, 0.0);
	if (mass != 0.0)
		invMass = 1.0 / mass;
	else
		invMass = 0.0;
}

// Copy constructor
Particle2D::Particle2D(const Particle2D& other)
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
Particle2D::Particle2D(Particle2D&& other) noexcept
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
Particle2D& Particle2D::operator=(const Particle2D& other) {
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
Particle2D& Particle2D::operator=(Particle2D&& other) noexcept {
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

Particle2D::~Particle2D()
{
}

void Particle2D::ApplyForce(const Vec2& force)
{
	netForces += force;
}

void Particle2D::ClearForces()
{
	netForces = Vec2(0.0f, 0.0f);
}

bool Aerolite::Particle2D::HasFiniteMass(void)
{
	return invMass != 0.0f;
}

void Particle2D::Integrate(const real dt)
{
	// Find the acceleration based on forces being applied.
	acceleration = netForces * invMass;

	// Integrate acceleration to find velocity;
	velocity += acceleration * dt;

	// Integrate velocity to find position.
	position += velocity * dt + (acceleration * dt * dt) / 2.0f;

	ClearForces();
}