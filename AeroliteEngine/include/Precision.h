#ifndef PRECISION_H
#define PRECISION_H

#include <cmath>

namespace Aerolite {

    //Defines a real number precision. Aerolite engine can be compiled in single or double-precision versions. By default, single
    //precision is provided.
#ifdef REAL_TYPE_DOUBLE
    typedef double real;
#else
    typedef float real;
#endif

    const real epsilon = 0.005f;

    inline bool AreEqual(real a, real b, real epsilon)
    {
        if (std::is_same_v<real, float>)
            return std::fabs(a - b) < epsilon;
        else if (std::is_same_v<real, double>)
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

    template<typename T>
    T make_real(double value) {
        return static_cast<T>(value);
    }

    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif

}

#endif