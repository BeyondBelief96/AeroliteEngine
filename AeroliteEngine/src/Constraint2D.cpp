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
	Aerolite::MatrixMxN Constraint2D::GetInvM() const {
		MatrixMxN invM(6,6);
		invM.Zero();
		invM.rows[0][0] = a.invMass;
		invM.rows[1][1] = a.invMass;
		invM.rows[2][2] = a.invI;

		invM.rows[3][3] = b.invMass;
		invM.rows[4][4] = b.invMass;
		invM.rows[5][5] = b.invI;
		return invM;
	}

	/// <summary>
	/// Creates velocity vector for constraint solving.
	/// </summary>
	/// <returns>Returns a Mat6x6 with all the inverse mass and inverse I of bodies "a" and "b"</returns>
	VecN Constraint2D::GetVelocities() const
	{
		VecN v(6);
		v.Zero();
		v[0] = a.velocity.x;
		v[1] = a.velocity.y;
		v[2] = a.angularVelocity;
		v[3] = b.velocity.x;
		v[4] = b.velocity.y;
		v[5] = b.angularVelocity;

		return v;
	}

	JointConstraint::JointConstraint(Aerolite::Body2D& a, Aerolite::Body2D& b, const Vec2& anchorPoint) : Constraint2D(a, b)
	{
		this->aPoint = a.WorldSpaceToLocalSpace(anchorPoint);
		this->bPoint = b.WorldSpaceToLocalSpace(anchorPoint);
		this->bias = 0.8;
		this->jacobian = MatrixMxN(1, 6);
	}

	void JointConstraint::Solve(void)
	{
		// Get the anchor points in world space.
		const Vec2 pa = a.LocalSpaceToWorldSpace(aPoint);
		const Vec2 pb = b.LocalSpaceToWorldSpace(bPoint);

		const Vec2 ra = pa - a.position; // vector from center of mass of body "a" to the anchor point in world space.
		const Vec2 rb = pb - b.position; // vector from center of mass of body "b" to the anchor point in world space.
		jacobian.Zero();
		// Load the joint constraint jacobian matrix.
		Vec2 J1 = (pa - pb) * 2.0;
		jacobian.rows[0][0] = J1.x; // coefficient for body "a" linear velocity.x;
		jacobian.rows[0][1] = J1.y; // coefficient for body "a" linear velocity.y;

		real J2 = 2.0 * (ra.Cross(pa - pb));
		jacobian.rows[0][2] = J2; // coefficient for body "a" angular velocity.

		Vec2 J3 = (pb - pa) * 2.0;
		jacobian.rows[0][3] = J3.x; // coefficient for body "b" linear velocity.x;
		jacobian.rows[0][4] = J3.y; // coefficient for body "b" linear velocity.y;

		real J4 = 2.0 * (rb.Cross(pb - pa));
		jacobian.rows[0][5] = J4; // coefficient for body "b" angular velocity.

		// Get the velocities vector
		const VecN v = GetVelocities();

		// Get inverse mass/moment matrix.
		MatrixMxN invM = GetInvM();

		// Compute lagrangian multiplier lambda which is the impulse magnitude to apply to "a" and "b".

		const MatrixMxN Jt = jacobian.Transpose();

		// Lambda = -(J * V + b) / (J * M^-1 * J^T)
		const VecN lambdaNumerator = (jacobian * v * -1.0); 
		const MatrixMxN lambdaDenominator = jacobian * invM * Jt;

		// (J * M^-1 * J^T) * Lambda = -(J * V + b) [Ax = B]
		VecN lambda = MatrixMxN::SolveGaussSeidel(lambdaDenominator, lambdaNumerator);

		// Compute the final impulses

		VecN impulses = Jt * lambda;

		// Apply the impulses to bodies a and b
		a.ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
		a.ApplyImpulseAngular(impulses[2]);

		b.ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
		b.ApplyImpulseAngular(impulses[5]);
	}

	void PenetrationConstraint::Solve(void)
	{
	}
}