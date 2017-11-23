#pragma once
#include "precision.hpp"
#include "Vertex3.hpp"
#include "Color.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class Image {

private:
    imp_uint _n_cols, _n_rows, _size;
    std::vector<imp_float> _pixel_values;
    std::vector<imp_float>_depth_buffer;

    const imp_float _eps_barycentric = -1.e-7f;

public:

	bool use_omp = false;
    
    Image(imp_uint n_cols, imp_uint n_rows);

    imp_uint getWidth() const;
    imp_uint getHeight() const;
    imp_float getAspectRatio() const;
    const imp_float* getRawPixelArray() const;
    
    Radiance getRadiance(imp_uint x, imp_uint y) const;
    Image& setRadiance(imp_uint x, imp_uint y, const Radiance& radiance);
    
    imp_float getDepth(imp_uint x, imp_uint y) const;
    Image& setDepth(imp_uint x, imp_uint y, imp_float depth);
    
    Image& initializeDepthBuffer(imp_float initial_value);
    Image& setBackgroundColor(const Color& color);

    Image& gammaEncode(imp_float normalization);
	Image& gammaEncodeApprox(imp_float normalization);
    Image& stretch();

    void drawLine(imp_float x0, imp_float y0, imp_float x1, imp_float y1, imp_float luminance);
    void drawTriangle(const Vertex3& v0, const Vertex3& v1, const Vertex3& v2);
};

} // Rendering3D
} // Impact
