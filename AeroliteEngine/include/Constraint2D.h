#ifndef CONSTRAINT2D_H
#define CONSTRAINT2D_H

#include "Body2D.h"
#include "MatrixMxN.h"
#include "VecN.h"

namespace Aerolite {
	class Constraint2D {
	public:
		Aerolite::Body2D& a; 
		Aerolite::Body2D& b;

		Vec2 aPoint; // The anchor point in A's local space.
		Vec2 bPoint; // The anchor point in B's local space.

		Constraint2D(Aerolite::Body2D& a, Aerolite::Body2D& b);
		virtual ~Constraint2D() = default;

		/// <summary>
		/// Creates inverse mass/moment matrix for constraint solving.
		/// </summary>
		/// <returns>matrix of inverse mass and inverse moment of inertia for bodies "a" and "b".</returns>
		Aerolite::MatrixMxN GetInvM() const;

		/// <summary>
		/// Creates linear and angular velocity vector for constraint solving.
		/// </summary>
		/// <returns>1x6 vector of linear and angular velocities of bodies "a" and "b".</returns>
		VecN GetVelocities() const;

		/// <summary>
		/// Solves the constraint.
		/// </summary>
		virtual void Solve(void) = 0;
	};

	class JointConstraint : public Constraint2D {
	private:
		MatrixMxN jacobian;
	public:
		real bias;
		JointConstraint() = default;
		JointConstraint(Aerolite::Body2D& a, Aerolite::Body2D& b, const Vec2& anchorPoint);

		/// <summary>
		/// Solves the joint constraint.
		/// </summary>
		void Solve(void) override;
	};

	class PenetrationConstraint : public Constraint2D {
		MatrixMxN jacobian;

		/// <summary>
		/// Solves the penetration constraint.
		/// </summary>
		/// <param name=""></param>
		void Solve(void) override;
	};
}

#endif