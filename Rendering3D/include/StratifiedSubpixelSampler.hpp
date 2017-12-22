#pragma once
#include "precision.hpp"
#include "SubpixelSampler.hpp"
#include "Point2.hpp"

namespace Impact {
namespace Rendering3D {

class StratifiedSubpixelSampler : public SubpixelSampler {

private:
	typedef Geometry2D::Point Point2;

protected:
	
	imp_uint _subgrid_resolution;
	imp_float _subgrid_cell_size;

	imp_uint _current_subgrid_i;
	imp_uint _current_subgrid_j;

	bool _sample_uniformly;

public:
	
	void initializeSampling(imp_uint n_samples);

	void setPixel(imp_uint i, imp_uint j);

	Point2 samplePixelPoint();
};

} // Rendering3D
} // Impact
