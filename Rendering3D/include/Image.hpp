#pragma once
#include "precision.hpp"
#include "image_util.hpp"
#include "Point3.hpp"
#include "Color.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class Image {

private:
	typedef Geometry3D::Point Point;

    imp_uint _width, _height, _n_elements, _size;
    imp_float* _pixel_buffer;
    imp_float* _depth_buffer;
	image_util::imp_ppm_data* _output_buffer;

    const imp_float _eps_barycentric = -1.e-7f;
	imp_float _depth_map_norm = 1;
	imp_float _depth_map_min_val = -1;

public:

	bool use_omp = false;
    
    Image(imp_uint width, imp_uint height);
	~Image();

    Image(const Image& other) = delete;
	Image& operator=(const Image& other) = delete;

    imp_uint getWidth() const;
    imp_uint getHeight() const;
    imp_float getAspectRatio() const;
    const imp_float* getRawPixelArray() const;
    
    Radiance getRadiance(imp_uint x, imp_uint y) const;
    void setRadiance(imp_uint x, imp_uint y, const Radiance& radiance);
    
	void accumulateRadiance(imp_uint x, imp_uint y, const Radiance& radiance);
	void normalizeAccumulatedRadiance(imp_uint x, imp_uint y, imp_float normalization);
    
    imp_float getDepth(imp_uint x, imp_uint y) const;
    void setDepth(imp_uint x, imp_uint y, imp_float depth);
    
    void initializeDepthBuffer(imp_float initial_value);

    void setBackgroundColor(const Color& color);

    void gammaEncode(imp_float normalization);
	void gammaEncodeApprox(imp_float normalization);
    void stretch();

	Color getApproxGammaEncodedColor(imp_uint x, imp_uint y, imp_float normalization) const;

    void drawLine(imp_float x0, imp_float y0, imp_float x1, imp_float y1, imp_float luminance);
    void drawTriangle(const Point& p0,
					  const Point& p1,
					  const Point& p2,
					  const Color& c0,
					  const Color& c1,
					  const Color& c2);
	
	void renderDepthMap(bool renormalize);
	
	void saveAsPPM(const std::string& filename);
};

} // Rendering3D
} // Impact
