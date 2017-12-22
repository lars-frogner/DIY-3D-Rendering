#include "SubpixelSampler.hpp"

namespace Impact {
namespace Rendering3D {
	
void SubpixelSampler::setPixel(imp_uint i, imp_uint j)
{
	_pixel_corner_x = static_cast<imp_float>(i);
	_pixel_corner_y = static_cast<imp_float>(j);
}

} // Rendering3D
} // Impact
