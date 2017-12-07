#include "math_util.hpp"
#include <random>

namespace Impact {
namespace math_util {
	
std::random_device seed;
std::mt19937 generator(seed());
std::uniform_real_distribution<imp_float> uniform_distribution(0.0f, 1.0f);

imp_int sign(imp_float val)
{
    return (imp_float(0) < val) - (val < imp_float(0));
}

imp_float random()
{
	return uniform_distribution(generator);
}

} // math_util
} // Impact
