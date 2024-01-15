#include "MatrixMxN.h"

namespace Aerolite {

	MatrixMxN::MatrixMxN() : M(0), N(0), rows(nullptr) {}

	MatrixMxN::MatrixMxN(int M, int N) : M(M), N(N) {
		rows = new VecN[M];
		for (int i = 0; i < M; i++)
			rows[i] = VecN(N);
	}

	MatrixMxN::MatrixMxN(const MatrixMxN& m) {
		*this = m;
	}

	MatrixMxN::~MatrixMxN() {
		delete[] rows;
	}

	void MatrixMxN::Zero() {
		for (int i = 0; i < M; i++)
			rows[i].Zero();
	}

	MatrixMxN MatrixMxN::Transpose() const {
		MatrixMxN result(N, M);
		for (int i = 0; i < M; i++)
			for (int j = 0; j < N; j++)
				result.rows[j][i] = rows[i][j];
		return result;
	}

	const MatrixMxN& MatrixMxN::operator = (const MatrixMxN& m) {
		M = m.M;
		N = m.N;
		rows = new VecN[M];
		for (int i = 0; i < M; i++)
			rows[i] = m.rows[i];
		return *this;
	}

	VecN MatrixMxN::operator * (const VecN& v) const {
		if (v.N != N)
			return v;
		VecN result(M);
		for (int i = 0; i < M; i++)
			result[i] = v.Dot(rows[i]);
		return result;
	}

	MatrixMxN MatrixMxN::operator * (const MatrixMxN& m) const {
		if (m.M != N && m.N != M)
			return m;
		MatrixMxN tranposed = m.Transpose();
		MatrixMxN result(M, m.N);
		for (int i = 0; i < M; i++)
			for (int j = 0; j < m.N; j++)
				result.rows[i][j] = rows[i].Dot(tranposed.rows[j]);
		return result;
	}

	VecN MatrixMxN::SolveGaussSeidel(const MatrixMxN& A, const VecN& b) {
		const int N = b.N;
		VecN X(N);
		X.Zero();

		// Iterate N times
		for (int iterations = 0; iterations < N; iterations++) {
			for (int i = 0; i < N; i++) {
				if (A.rows[i][i] != 0.0f) {
					X[i] += (b[i] / A.rows[i][i]) - (A.rows[i].Dot(X) / A.rows[i][i]);
				}
			}
		}
		return X;
	}

}