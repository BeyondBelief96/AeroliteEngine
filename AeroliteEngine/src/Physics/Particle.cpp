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