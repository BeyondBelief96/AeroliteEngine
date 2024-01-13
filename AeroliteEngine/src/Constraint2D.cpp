#include "Constraint2D.h"

namespace Aerolite {

	/// <summary>
	/// Creates inverse mass matrix.
	/// </summary>
	/// <returns>Returns a Mat6x6 with all the inverse mass and inverse I of bodies "a" and "b"</returns>
	Aerolite::MatrixMxN<6, 6> Constraint2D::GetInvM() const {
		MatrixMxN<6, 6> invM;

		invM(0, 0) = a.invMass;
		invM(1, 1) = a.invMass;
		invM(2, 2) = a.invI;

		invM(3, 3) = b.invMass;
		invM(4, 4) = b.invMass;
		invM(5, 5) = b.invI;
		return invM;
	}

	/// <summary>
	/// Creates velocity vector for constraint solving.
	/// </summary>
	/// <returns>Returns a Mat6x6 with all the inverse mass and inverse I of bodies "a" and "b"</returns>
	VecN<6> Constraint2D::GetVelocities() const
	{
		VecN<6> v;

		v[0] = a.velocity.x;
		v[1] = a.velocity.y;
		v[2] = a.angularVelocity;
		v[3] = b.velocity.x;
		v[4] = b.velocity.y;
		v[5] = b.angularVelocity;

		return v;
	}
}