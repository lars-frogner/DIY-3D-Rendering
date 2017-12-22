#include "StratifiedSubpixelSampler.hpp"
#include "math_util.hpp"
#include <cassert>
#include <cmath>

namespace Impact {
namespace Rendering3D {

void StratifiedSubpixelSampler::initializeSampling(imp_uint n_samples)
{
	assert(n_samples > 0);

	_subgrid_resolution = static_cast<imp_uint>(floor(sqrt(static_cast<imp_float>(n_samples))));
	_subgrid_cell_size = 1.0f/_subgrid_resolution;
}

void StratifiedSubpixelSampler::setPixel(imp_uint i, imp_uint j)
{
	_pixel_corner_x = static_cast<imp_float>(i);
	_pixel_corner_y = static_cast<imp_float>(j);
	
	_current_subgrid_i = 0;
	_current_subgrid_j = 0;

	_sample_uniformly = false;
}

Geometry2D::Point StratifiedSubpixelSampler::samplePixelPoint()
{
	Point2 sample;

	if (_sample_uniformly)
	{
		sample.moveTo(_pixel_corner_x + math_util::random(),
					  _pixel_corner_y + math_util::random());
	}
	else
	{
		sample.moveTo(_pixel_corner_x + (_current_subgrid_i + math_util::random())*_subgrid_cell_size,
					  _pixel_corner_y + (_current_subgrid_j + math_util::random())*_subgrid_cell_size);

		_current_subgrid_i++;

		if (_current_subgrid_i == _subgrid_resolution)
		{
			_current_subgrid_i = 0;
			_current_subgrid_j++;

			if (_current_subgrid_j == _subgrid_resolution)
				_sample_uniformly = true;
		}
	}

	return sample;
}

} // Rendering3D
} // Impact
