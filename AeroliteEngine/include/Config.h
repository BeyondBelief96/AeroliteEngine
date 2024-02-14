#ifndef AERO_CONFIG_H
#define AERO_CONFIG_H

namespace Aerolite
{

/**
 * @brief Uncomment this and comment out REAL_TYPE_DOUBLE for single floating point precision. 
 */
//#define REAL_TYPE_FLOAT
/**
* @brief Uncomment this and comment out REAL_TYPE_DOUBLE for single floating point precision.
*/
 #define REAL_TYPE_DOUBLE

/**
 * \brief Uncomment if you want static objects to be included in collision checks.
 */
#define CHECK_STATIC_COLLISIONS

#define BROAD_PHASE_BRUTE_FORCE
#define BROAD_PHASE_SHG

/*
 * Aerolite Engine Typedefs
 */
	typedef int8_t aero_int8;
	typedef int16_t aero_int16;
	typedef int32_t aero_int32;
	typedef int64_t aero_int64;
	typedef uint8_t aero_uint8;
	typedef uint16_t aero_uint16;
	typedef uint32_t aero_uint32;
	typedef uint64_t aero_uint64;
}

#endif


