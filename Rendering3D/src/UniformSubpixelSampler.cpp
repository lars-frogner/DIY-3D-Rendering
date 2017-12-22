#include "UniformSubpixelSampler.hpp"
#include "math_util.hpp"

namespace Impact {
namespace Rendering3D {

void UniformSubpixelSampler::initializeSampling(imp_uint n_samples)
{
	// No initialization required
}

Geometry2D::Point UniformSubpixelSampler::samplePixelPoint()
{
	return Point2(_pixel_corner_x + math_util::random(),
				  _pixel_corner_y + math_util::random());
}

} // Rendering3D
} // Impact
