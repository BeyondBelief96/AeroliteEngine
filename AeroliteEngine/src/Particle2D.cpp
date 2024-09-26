#include "Particle2D.h"
#include <iostream>

namespace Aerolite {
	Particle2D::Particle2D(const real x, const real y, const real mass) : position(x, y), mass(mass)
	{
		radius = 4.0;
		velocity = AeroVec2(0.0, 0.0);
		acceleration = AeroVec2(0.0, 0.0);
		netForces = AeroVec2(0.0, 0.0);
		if (mass != 0.0)
			invMass = make_real<real>(1.0) / mass;
		else
			invMass = 0.0;
	}

	void Particle2D::ApplyForce(const AeroVec2& force)
	{
		netForces += force;
	}

	void Particle2D::ClearForces()
	{
		netForces = AeroVec2(0.0f, 0.0f);
	}

	bool Particle2D::HasFiniteMass(void) const
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
}
