#pragma once
#include "precision.hpp"
#include "SubpixelSampler.hpp"
#include "Point2.hpp"

namespace Impact {
namespace Rendering3D {

class UniformSubpixelSampler : public SubpixelSampler {

private:
	typedef Geometry2D::Point Point2;

public:
	
	void initializeSampling(imp_uint n_samples);

	Point2 samplePixelPoint();
};

} // Rendering3D
} // Impact
