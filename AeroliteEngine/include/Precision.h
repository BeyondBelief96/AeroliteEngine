#ifndef PRECISION_H
#define PRECISION_H

#include <cmath>

#include "Config.h"

namespace Aerolite {


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    //Defines a real number precision. Aerolite engine can be compiled in single or double-precision versions. By default, single
    //precision is provided.
#ifdef REAL_TYPE_DOUBLE
    typedef double real;
#else
    typedef float real;
#endif

    const real EPSILON = 0.005f;

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
        const real t = a;
        a = b;
        b = t;
    }

    /**
     * \brief Takes the sqrt of the 'real' typedef depending if it's currently
     * defined as a float or double.
     * \param a The value to take the sqrt of.
     * \return Returns the sqrt of the value of a.
     */
    inline real RealSqrt(const real a)
    {
        if (std::is_same_v < real, float>)
            return std::sqrtf(a);
        else if (std::is_same_v<real, double>)
            return std::sqrt(a);
    }

	/**
    * \brief Takes the sin of the 'real' typedef depending if it's currently
    * defined as a float or double.
    * \param a The value to take the sin of.
    * \return Returns the sin of the value of a.
    */
    inline real RealSin(const real a)
    {
        if (std::is_same_v < real, float>)
            return std::sinf(a);
        else if (std::is_same_v<real, double>)
            return std::sin(a);
    }

    /**
    * \brief Takes the cos of the 'real' typedef depending if it's currently
    * defined as a float or double.
    * \param a The value to take the cos of.
    * \return Returns the cos of the value of a.
    */
    inline real RealCos(const real a)
    {
        if (std::is_same_v < real, float>)
            return std::cosf(a);
        else if (std::is_same_v<real, double>)
            return std::cos(a);
    }

    /**
    * @brief Raises a base to a given power.
    * \param base The value to take the cos of.
    * @param power The value to raise the base to.
    * \return Returns the result of base ^ power.
    */
    inline real RealPow(const real base, const real power)
    {
        if (std::is_same_v < real, float>)
            return std::powf(base, power);
        else if (std::is_same_v<real, double>)
            return std::pow(base, power);
    }

    template<typename T>
    T make_real(double value) {
        return static_cast<T>(value);
    }

    /**
     * \brief Combines two 16-bit integers into a 32 bit integer.
     * \param x The first integer
     * \param y The second integer
     * \return The two integers combined into a 32 bit integer
     */
	inline aero_uint32 ComputeIdPair(const aero_uint16 x, const aero_uint16 y) {
        // https://stackoverflow.com/a/919631
        return ((aero_int32)x << 16) | (aero_int32)y;
    }

}

#endif