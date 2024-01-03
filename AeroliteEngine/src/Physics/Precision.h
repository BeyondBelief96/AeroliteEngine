#ifndef PRECISION_H
#define PRECISION_H

#include <cmath>

namespace Aerolite {

    ///Defines a real number precision. Aerolite engine can be compiled in single or double-precision versions. By default, single
    ///precision is provided.
    typedef float real;

    const float epsilon = 0.005f;

    inline bool AreFloatsEqual(float a, float b, float epsilon) 
    {
        return std::fabs(a - b) < epsilon;
    }

    
}

#endif