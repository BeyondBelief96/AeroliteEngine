#ifndef PRECISION_H
#define PRECISION_H

#include <cmath>

namespace Aerolite {

    //Defines a real number precision. Aerolite engine can be compiled in single or double-precision versions. By default, single
    //precision is provided.
    typedef float real;

    const real epsilon = 0.005f;

    inline bool AreEqual(Aerolite::real a, Aerolite::real b, Aerolite::real epsilon)
    {
        if (std::is_same<Aerolite::real, float>::value)
            return std::fabs(a - b) < epsilon;
        else if (std::is_same<Aerolite::real, double>::value)
            return std::abs(a - b) < epsilon;
    }

    /// <summary>
    /// Swaps the values a and b.
    /// </summary>
    /// <param name="a">The value to take on the value of b.</param>
    /// <param name="b">The value to take on the value of a.</param>
    inline void Swap(real a, real b)
    {
        real t = a;
        a = b;
        b = t;
    }

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

}

#endif