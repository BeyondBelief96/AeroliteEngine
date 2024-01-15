#include "VecN.h"

namespace Aerolite {

	VecN::VecN() : N(0), data(nullptr) {}

	VecN::VecN(int N) : N(N) {
		data = new real[N];
	}

	VecN::VecN(const VecN& v) {
		N = v.N;
		data = new real[N];
		for (int i = 0; i < N; i++)
			data[i] = v.data[i];
	}

	VecN::~VecN() {
		delete[] data;
	}

	void VecN::Zero() {
		for (int i = 0; i < N; i++)
			data[i] = 0.0f;
	}

	real VecN::Dot(const VecN& v) const {
		real sum = 0.0f;
		for (int i = 0; i < N; i++)
			sum += data[i] * v.data[i];
		return sum;
	}

	VecN& VecN::operator = (const VecN& v) {
		delete[] data;
		N = v.N;
		data = new real[N];
		for (int i = 0; i < N; i++)
			data[i] = v.data[i];
		return *this;
	}

	VecN VecN::operator * (real n) const {
		VecN result = *this;
		result *= n;
		return result;
	}

	VecN VecN::operator + (const VecN& v) const {
		VecN result = *this;
		for (int i = 0; i < N; i++)
			result.data[i] += v.data[i];
		return result;
	}

	VecN VecN::operator - (const VecN& v) const {
		VecN result = *this;
		for (int i = 0; i < N; i++)
			result.data[i] -= v.data[i];
		return result;
	}

	const VecN& VecN::operator *= (real n) {
		for (int i = 0; i < N; i++)
			data[i] *= n;
		return *this;
	}

	const VecN& VecN::operator += (const VecN& v) {
		for (int i = 0; i < N; i++)
			data[i] += v.data[i];
		return *this;
	}

	const VecN& VecN::operator -= (const VecN& v) {
		for (int i = 0; i < N; i++)
			data[i] -= v.data[i];
		return *this;
	}

	real VecN::operator [] (const int index) const {
		return data[index];
	}

	real& VecN::operator [] (const int index) {
		return data[index];
	}

}