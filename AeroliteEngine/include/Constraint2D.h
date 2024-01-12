#ifndef CONSTRAINT2D_H
#define CONSTRAINT2D_H

#include "Body2D.h"

namespace Aerolite {
	class Constraint2D {
	public:
		Aerolite::Body2D& a;
		Aerolite::Body2D& b;
		void Solve();
	};
}


#endif