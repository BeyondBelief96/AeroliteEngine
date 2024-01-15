#include "Constraint2D.h"


namespace Aerolite {

	/// <summary>
	/// Default constructor
	/// </summary>
	/// <param name="a">The first body used for the constraint.</param>
	/// <param name="b">The second body used for the constraint.</param>
	Constraint2D::Constraint2D(Aerolite::Body2D& a, Aerolite::Body2D& b) : a(a), b(b)
	{
	}

	/// <summary>
	/// Creates inverse mass/moment matrix.
	/// </summary>
	/// <returns>Returns a Mat6x6 with all the inverse mass and inverse I of bodies "a" and "b"</returns>
	Aerolite::MatrixMxN<6,6> Constraint2D::GetInvM() const {
		MatrixMxN<6, 6> invM;
		invM.Zero();
		invM[0][0] = a.invMass;
		invM[1][1] = a.invMass;
		invM[2][2] = a.invI;

		invM[3][3] = b.invMass;
		invM[4][4] = b.invMass;
		invM[5][5] = b.invI;
		return invM;
	}

	/// <summary>
	/// Creates velocity vector for constraint solving.
	/// </summary>
	/// <returns>Returns a Mat6x6 with all the inverse mass and inverse I of bodies "a" and "b"</returns>
	MatrixMxN<6,1>Constraint2D::GetVelocities() const
	{
		MatrixMxN<6,1> v;
		v[0][0] = a.velocity.x;
		v[1][0] = a.velocity.y;
		v[2][0] = a.angularVelocity;
		v[3][0] = b.velocity.x;
		v[4][0] = b.velocity.y;
		v[5][0] = b.angularVelocity;

		return v;
	}

	JointConstraint::JointConstraint(Aerolite::Body2D& a, Aerolite::Body2D& b, const Vec2& anchorPoint) : Constraint2D(a, b)
	{
		this->aPoint = a.WorldSpaceToLocalSpace(anchorPoint);
		this->bPoint = b.WorldSpaceToLocalSpace(anchorPoint);
		this->bias = 0.8;
		this->jacobian = MatrixMxN<1,6>();
		this->cachedLambda = 0;
	}

	void JointConstraint::PreSolve(void)
	{
		// Get the anchor points in world space.
		const Vec2 pa = a.LocalSpaceToWorldSpace(aPoint);
		const Vec2 pb = b.LocalSpaceToWorldSpace(bPoint);

		const Vec2 ra = pa - a.position; // vector from center of mass of body "a" to the anchor point in world space.
		const Vec2 rb = pb - b.position; // vector from center of mass of body "b" to the anchor point in world space.
		jacobian.Zero();
		// Load the joint constraint jacobian matrix.
		Vec2 J1 = (pa - pb) * 2.0;
		jacobian[0][0] = J1.x; // coefficient for body "a" linear velocity.x;
		jacobian[0][1] = J1.y; // coefficient for body "a" linear velocity.y;

		real J2 = 2.0 * (ra.Cross(pa - pb));
		jacobian[0][2] = J2; // coefficient for body "a" angular velocity.

		Vec2 J3 = (pb - pa) * 2.0;
		jacobian[0][3] = J3.x; // coefficient for body "b" linear velocity.x;
		jacobian[0][4] = J3.y; // coefficient for body "b" linear velocity.y;

		real J4 = 2.0 * (rb.Cross(pb - pa));
		jacobian[0][5] = J4; // coefficient for body "b" angular velocity.

		auto impulses = jacobian.Transpose() * cachedLambda;

		a.ApplyImpulseLinear(Vec2(impulses[0][0], impulses[1][0]));
		a.ApplyImpulseAngular(impulses[2][0]);

		b.ApplyImpulseLinear(Vec2(impulses[3][0], impulses[4][0]));
		b.ApplyImpulseAngular(impulses[5][0]);
	}

	void JointConstraint::Solve(void)
	{
		// Get the velocities vector
		const MatrixMxN v = GetVelocities();

		// Get inverse mass/moment matrix.
		MatrixMxN invM = GetInvM();

		// Compute lagrangian multiplier lambda which is the impulse magnitude to apply to "a" and "b".
		const MatrixMxN Jt = jacobian.Transpose();

		// Lambda = -(J * V + b) / (J * M^-1 * J^T)
		const MatrixMxN lambdaNumerator = (jacobian * v * -1.0); 
		const MatrixMxN lambdaDenominator = jacobian * invM * Jt;

		// (J * M^-1 * J^T) * Lambda = -(J * V + b) [Ax = B]
		auto lambda = MatrixMxN<1,1>::SolveGaussSeidel(lambdaDenominator, VecN(lambdaNumerator[0]));
		cachedLambda += lambda[0];

		// Compute the final impulses
		auto impulses = Jt * lambda[0];

		// Apply the impulses to bodies a and b
		a.ApplyImpulseLinear(Vec2(impulses[0][0], impulses[1][0]));
		a.ApplyImpulseAngular(impulses[2][0]);

		b.ApplyImpulseLinear(Vec2(impulses[3][0], impulses[4][0]));
		b.ApplyImpulseAngular(impulses[5][0]);
	}

	void JointConstraint::PostSolve(void)
	{

	}

	void PenetrationConstraint::Solve(void)
	{
	}
}