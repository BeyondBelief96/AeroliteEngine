#ifndef CONSTRAINT2D_H
#define CONSTRAINT2D_H

#include "Body2D.h"
#include "MatrixMxN.h"
#include "VecN.h"

namespace Aerolite {
	class Constraint2D {
	public:
		Body2D& a;
		Body2D& b;

		AeroVec2 aPoint; // Constraint specific point in A's local space.
		AeroVec2 bPoint; // Constraint specific point in B's local space.

		Constraint2D(Body2D& a, Body2D& b);
		virtual ~Constraint2D() = default;

		/// <summary>
		/// Creates inverse mass/moment matrix for constraint solving.
		/// </summary>
		/// <returns>matrix of inverse mass and inverse moment of inertia for bodies "a" and "b".</returns>
		MatrixMxN<6, 6> GetInvM() const;

		/// <summary>
		/// Creates linear and angular velocity vector for constraint solving.
		/// </summary>
		/// <returns>1x6 vector of linear and angular velocities of bodies "a" and "b".</returns>
		MatrixMxN<6,1> GetVelocities() const;


		virtual void PreSolve(const real dt) = 0;

		/// <summary>
		/// Solves the constraint.
		/// </summary>
		virtual void Solve(void) = 0;

		virtual void PostSolve(void) = 0;
	};

	class JointConstraint : public Constraint2D {
	private:
		MatrixMxN<1,6> jacobian;
		real cachedLambda;
		real bias;
	public:

		JointConstraint() = default;
		JointConstraint(Body2D& a, Body2D& b, const AeroVec2& anchorPoint);

		void PreSolve(const real dt) override;
		void Solve(void) override;
		void PostSolve(void) override;
	};

	class PenetrationConstraint : public Constraint2D {
	private:
		MatrixMxN<2,6> jacobian;
		VecN<2> cachedLambda;
		real bias;
		AeroVec2 normal; // Collision normal in A's local space.
		real friction; // Friction coefficient between the two bodies.
	public:
		PenetrationConstraint() = default;
		PenetrationConstraint(
			Body2D& a,
			Body2D& b,
			const AeroVec2& aCollisionPoint,
			const AeroVec2& bCollisionPoint, 
			const AeroVec2& collisionNormal);
		void PreSolve(const real dt) override;
		void Solve(void) override;
		void PostSolve(void) override;
	};
}

#endif