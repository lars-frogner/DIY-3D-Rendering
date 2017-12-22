#pragma once
#include "precision.hpp"
#include "Point2.hpp"

namespace Impact {
namespace Rendering3D {

class SubpixelSampler {

private:
	typedef Geometry2D::Point Point2;

protected:
	imp_float _pixel_corner_x;
	imp_float _pixel_corner_y;

public:
	
	virtual void initializeSampling(imp_uint n_samples) = 0;

	virtual void setPixel(imp_uint i, imp_uint j);

	virtual Point2 samplePixelPoint() = 0;
};

} // Rendering3D
} // Impact
