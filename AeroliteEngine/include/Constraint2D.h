#ifndef CONSTRAINT2D_H
#define CONSTRAINT2D_H

#include "Body2D.h"
#include "MatMxN.h"
#include "VecN.h"

namespace Aerolite {
	class Constraint2D {
	public:
		Aerolite::Body2D& a;
		Aerolite::Body2D& b;

		virtual ~Constraint2D() = default;

		Aerolite::MatrixMxN<6, 6> GetInvM() const;
		VecN<6> GetVelocities() const;
		virtual void Solve() = 0;
	};

	class DistanceConstraint : public Constraint2D {
		void Solve() override;
	};

	class PenetrationConstraint : public Constraint2D {
		void Solve() override;
	};
}




#endif