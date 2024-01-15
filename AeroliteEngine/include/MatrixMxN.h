#ifndef MATMN_H
#define MATMN_H

#include "VecN.h"

namespace Aerolite {
    struct MatrixMxN {
        int M;      // rows
        int N;      // cols
        VecN* rows; // the rows of the matrix with N columns inside

        MatrixMxN();
        MatrixMxN(int M, int N);
        MatrixMxN(const MatrixMxN& m);
        ~MatrixMxN();

        void Zero();
        MatrixMxN Transpose() const;

        const MatrixMxN& operator = (const MatrixMxN& m);  // m1 = m2
        VecN operator * (const VecN& v) const;     // m1 * v
        MatrixMxN operator * (const MatrixMxN& m) const;   // m1 * m2

        static VecN SolveGaussSeidel(const MatrixMxN& A, const VecN& b);
    };
}


#endif
