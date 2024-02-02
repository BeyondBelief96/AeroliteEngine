#ifndef AERO_QUAT_H
#define AERO_QUAT_H
#include "AeroVec3.h"
#include "Precision.h"

namespace Aerolite
{
	/**
	 * \brief Quaternion object that holds three degrees of freedom for orientation.
	 */
	class Quaternion
	{
	public:
		union
		{
			struct
			{
				/**
				 * \brief Holds the real component of the quaternion.
				 */
				real r;

				/**
				 * \brief Holds the first complex component of the quaternion.
				 */
				real i;

				/**
				 * \brief Holds the second complex component of the quaternion.
				 */
				real j;

				/**
				 * \brief Holds the third complex component of the quaternion.
				 */
				real k;
			};


			/**
			 * \brief Holds the quaternion data in array form.
			 */
			real data[4];
		};

		Quaternion() = default;
		~Quaternion() = default;

		// Constructor for directly setting components
		Quaternion(const real r, const real i, const real j, const real k) : r(r), i(i), j(j), k(k) {}

		/**
		 * \brief // Creates a quaternion representing a rotation around a unit vector axis by an angle in radians.
		 * \param axis The axis to rotate by.
		 * \param angleRadians The amount to rotate by in radians.
		 * \return A quaternion constructed from the given axis and angle.
		 */
		static Quaternion FromAxisAngle(const AeroVec3& axis, const real angleRadians)
		{
			const real halfAngle = angleRadians * 0.5;
			const real s = RealSin(halfAngle);
			return Quaternion(RealCos(halfAngle), axis.x * s, axis.y * s, axis.z * s);
		}

		/**
		 * \brief Takes a quaternion as an argument, and add component wise to this quaternion.
		 * \param q The quaternion to add to this quaternion.
		 */
		void Add(const Quaternion& q)
		{
			r += q.r;
			i += q.i;
			j += q.j;
			k += q.k;
		}

		/**
		 * \brief Takes a quaternion as an argument, and subtracts it component wise from this quaternion.
		 * \param q The quaternion to subtract from this quaternion.
		 */
		void Subract(const Quaternion q)
		{
			r -= q.r;
			i -= q.i;
			j -= q.j;
			k -= q.k;
		}

		/**
		 * \brief Returns the conjugate of the quaternion.
		 * \return Returns the conjugate of the quaternion.
		 */
		Quaternion Conjugate() const
		{
			return Quaternion(r, -i, -j, -k);
		}

		/**
		 * \brief Returns the magnitude of the quaternion.
		 * \return Returns the magnitude of the quaternion.
		 */
		real Magnitude() const
		{
			return RealSqrt(r * r + i * i + j * j + k * k);
		}

		/**
		 * \brief Returns the squared magnitude of the quaternion.
		 * \return Returns the squared magnitude of the quaternion.
		 */
		real MagnitudeSquared() const
		{
			return r * r + i * i + j * j + k * k;
		}

		void Normalize()
		{
			real d = MagnitudeSquared();
			// Check for zero-length quaternion, and use the no-rotation
			// quaternion in that case.
			if (AreEqual(d, 0, EPSILON)) {
				r = 1;
				return;
			}
			d = static_cast<real>(1.0) / RealSqrt(d);
			r *= d;
			i *= d;
			j *= d;
			k *= d;
		}

		/**
		 * \brief Rotates this quaternion by the given AeroVec3.
		 * \param vector The vector in scaled axis representation to rotate this quaternion by.
		 */
		void RotateByVector(const AeroVec3& vector)
		{
			const Quaternion q(0, vector.x, vector.y, vector.z);
			(*this) *= q;
		}

		void AddScaledVector(const AeroVec3& vector, const real scale)
		{
			// First, create a quaternion representing the angular velocity vector
			// The real part is 0 because we're representing a pure rotation
			Quaternion p(0, vector.x * scale, vector.y * scale, vector.z * scale);

			// This quaternion is then scaled by 0.5 to integrate over the timestep
			// (as per the quaternion derivative formula for rotation)
			p = p * static_cast<real>(0.5);

			// Now, we multiply the current orientation quaternion by omega
			// Note: The correct operation is to pre-multiply the current quaternion (this)
			// by the small rotation quaternion (omega) to apply the rotation correctly.
			const Quaternion q = p * (*this);

			// Update the current quaternion components with the result
			r += q.r;
			i += q.i;
			j += q.j;
			k += q.k;

			// Normalize the quaternion to correct any drift from being a unit quaternion
			Normalize();
		}

		/**
		 * \brief Operator overload to add two quaternions together component wise.
		 * \param q The quaternion to add to this quaternion.
		 */
		Quaternion operator +(const Quaternion q) const noexcept
		{
			Quaternion p;
			p.r = r + q.r;
			p.i = i + q.i;
			p.j = j + q.j;
			p.k = k + q.k;
			return p;
		}

		/**
		 * \brief Operator overload to subtract two quaternions together component wise.
		 * \param q The quaternion to subtract from this quaternion.
		 */
		Quaternion operator -(const Quaternion q) const noexcept
		{
			Quaternion p;
			p.r = r - q.r;
			p.i = i - q.i;
			p.j = j - q.j;
			p.k = k - q.k;
			return p;
		}

		/**
		 * \brief Multiplies this quaternion by the quaternion p and returns the result.
		 * \param p The quaternion to multiply this quaternion by
		 * \return A new quaternion which is the result of this * p
		 */
		Quaternion operator *(const Quaternion p) const
		{
			Quaternion q;
			q.r = r * p.r - i * p.i -
				j * p.j - k * p.k;
			q.i = r * p.i + i * p.r +
				j * p.k - k * p.j;
			q.j = r * p.j + j * p.r +
				k * p.i - i * p.k;
			q.k = r * p.k + k * p.r +
				i * p.j - j * p.i;
			return q;
		}

		/**
		 * \brief Multiplies this quaternion by p, and stores the result in this quaternion.
		 * \param p The quaternion to multiply this quaternion by.
		 */
		void operator *=(const Quaternion& p)
		{
			const Quaternion q = *this;
			r = q.r * p.r - q.i * p.i -
				q.j * p.j - q.k * p.k;
			i = q.r * p.i + q.i * p.r +
				q.j * p.k - q.k * p.j;
			j = q.r * p.j + q.j * p.r +
				q.k * p.i - q.i * p.k;
			k = q.r * p.k + q.k * p.r +
				q.i * p.j - q.j * p.i;
		}

		// Overload for multiplying a quaternion by a scalar
		Quaternion operator *(const real n) const
		{
			return Quaternion(r * n, i * n, j * n, k * n);
		}

		// Overload for multiplying a quaternion by a scalar
		void operator *=(const real n)
		{
			r *= n;
			i*= n;
			j*= n;
			k*= n;
		}
	};
}

#endif

