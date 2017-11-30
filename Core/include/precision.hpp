#pragma once
#define _USE_MATH_DEFINES
#include <limits>
#include <ctime>

//#pragma warning(disable: 4305)

namespace Impact {
	
typedef float imp_float;
typedef double imp_double;
typedef int imp_int;
typedef unsigned long imp_uint;
typedef unsigned long long imp_ulong;

const imp_float IMP_FLOAT_INF = std::numeric_limits<imp_float>::infinity();
const imp_float IMP_FLOAT_MAX = std::numeric_limits<imp_float>::max();
const imp_float IMP_FLOAT_MIN = std::numeric_limits<imp_float>::min();

const imp_float IMP_PI = static_cast<imp_float>(M_PI);
const imp_float IMP_TWO_PI = 2*IMP_PI;
const imp_float IMP_FOUR_PI = 4*IMP_PI;
const imp_float IMP_DEG_TO_RAD = IMP_PI/180;

const imp_float IMP_RAND_NORM = 1/static_cast<imp_float>(RAND_MAX);

const imp_double IMP_SECS_PER_CLOCK = 1/static_cast<imp_double>(CLOCKS_PER_SEC);

} // Impact
