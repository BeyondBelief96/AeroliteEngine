#ifndef VECN_H
#define VECN_H

#include <stdexcept>
#include <cassert>
#include "Precision.h"

namespace Aerolite {

    struct VecN {
        int N;
        real* data;

        VecN();
        VecN(int N);
        VecN(const VecN& v);
        ~VecN();

        void Zero();                              // v1.Zero()
        real Dot(const VecN& v) const;            // v1.Dot(v2)

        VecN& operator = (const VecN& v);          // v1 = v2
        VecN operator + (const VecN& v) const;     // v1 + v2
        VecN operator - (const VecN& v) const;     // v1 - v2
        VecN operator * (const real n) const;     // v1 * n
        const VecN& operator += (const VecN& v);   // v1 += v2
        const VecN& operator -= (const VecN& v);   // v1 -= v2
        const VecN& operator *= (const real n);   // v1 *= n
        real operator [] (const int index) const; // v1[index]
        real& operator [] (const int index);      // v1[index]
    };
}

#endif // VECN_H
