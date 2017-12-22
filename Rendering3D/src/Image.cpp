#include "Image.hpp"
#include <omp.h>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <string>
#include <cstdio>

namespace Impact {
namespace Rendering3D {

Image::Image(imp_uint width, imp_uint height)
    : _width(width), _height(height), _n_elements(width*height), _size(3*width*height),
      _pixel_buffer(new imp_float[3*width*height]),
	  _depth_buffer(nullptr),
	  _output_buffer(nullptr) {}

Image::~Image()
{
	delete _pixel_buffer;

	if (_depth_buffer)
		delete _depth_buffer;

	if (_output_buffer)
		delete _output_buffer;
}

void Image::drawLine(imp_float x0, imp_float y0, imp_float x1, imp_float y1, imp_float luminance)
{
    // Bresenham's line algorithm

    imp_uint idx;

    imp_int x = static_cast<imp_int>(round(x0));
    imp_int y = static_cast<imp_int>(round(y0));

    imp_int w = static_cast<imp_int>(round(x1)) - x;
    imp_int h = static_cast<imp_int>(round(y1)) - y;

    imp_int dx0 = 0, dy0 = 0, dx1 = 0, dy1 = 0;

    if (w < 0)
    {
        dx0 = -1;
        dx1 = -1;
    }
    else if (w > 0)
    {
        dx0 = 1;
        dx1 = 1;
    }

    if      (h < 0) dy0 = -1;
    else if (h > 0) dy0 = 1;

    imp_int longest = abs(w);
    imp_int shortest = abs(h);
    imp_int temp;

    if (!(longest > shortest))
    {
        temp = longest;
        longest = shortest;
        shortest = temp;
        
        if (h < 0) dy1 = -1;
        else if (h > 0) dy1 = 1;
        
        dx1 = 0;
    }

    int numerator = longest >> 1;

    for (imp_int i = 0; i <= longest; i++)
    {
        idx = y*_width + x;

		if (idx >= _n_elements)
			return;

        idx *= 3;
        _pixel_buffer[idx] = luminance;
        _pixel_buffer[idx + 1] = luminance;
        _pixel_buffer[idx + 2] = luminance;

        numerator += shortest;
        if (!(numerator < longest))
        {
            numerator -= longest;
            x += dx0;
            y += dy0;
        }
        else
        {
            x += dx1;
            y += dy1;
        }
    }
}

void Image::drawTriangle(const Point& p0,
						 const Point& p1,
						 const Point& p2,
						 const Color& c0,
						 const Color& c1,
						 const Color& c2)
{
	assert(_depth_buffer);

    imp_int x0_i = static_cast<imp_int>(round(p0.x));
    imp_int y0_i = static_cast<imp_int>(round(p0.y));
    imp_int x1_i = static_cast<imp_int>(round(p1.x));
    imp_int y1_i = static_cast<imp_int>(round(p1.y));
    imp_int x2_i = static_cast<imp_int>(round(p2.x));
    imp_int y2_i = static_cast<imp_int>(round(p2.y));

    imp_int x_min = std::min(x0_i, std::min(x1_i, x2_i));
    imp_int x_max = std::max(x0_i, std::max(x1_i, x2_i));
    imp_int y_min = std::min(y0_i, std::min(y1_i, y2_i));
    imp_int y_max = std::max(y0_i, std::max(y1_i, y2_i));

	if (x_min < 0 || x_max >= static_cast<imp_int>(_width) || y_min < 0 || y_max >= static_cast<imp_int>(_height))
		return;

    //assert(x_min >= 0 && x_max < _width);
    //assert(y_min >= 0 && y_max < _height);

    imp_int dx1 = x1_i - x0_i, dy1 = y1_i - y0_i;
    imp_int dx2 = x2_i - x0_i, dy2 = y2_i - y0_i;
    imp_int dxp, dyp;
    
    imp_float factor;
    imp_float alpha, beta, gamma;
    imp_float z;

    imp_int x, y;
    imp_uint idx;

    for (y = y_min; y <= y_max; y++) {
        for (x = x_min; x <= x_max; x++)
        {
            dxp = x - x0_i; dyp = y - y0_i;

            factor = 1/static_cast<imp_float>(dx1*dy2 - dx2*dy1);

            beta = (dxp*dy2 - dx2*dyp)*factor;
            gamma = (dx1*dyp - dxp*dy1)*factor;
            alpha = 1 - beta - gamma;

            if (alpha > _eps_barycentric && beta > _eps_barycentric && gamma > _eps_barycentric)
            {
                idx = y*_width + x;
                z = alpha*p0.z + beta*p1.z + gamma*p2.z;

                if (z > _depth_buffer[idx])
                {
                    _depth_buffer[idx] = z;

                    idx *= 3;
                    _pixel_buffer[idx] = alpha*c0.r + beta*c1.r + gamma*c2.r;
                    _pixel_buffer[idx + 1] = alpha*c0.g + beta*c1.g + gamma*c2.g;
                    _pixel_buffer[idx + 2] = alpha*c0.b + beta*c1.b + gamma*c2.b;
                }
            }
        }
    }
}

Radiance Image::getRadiance(imp_uint x, imp_uint y) const
{
    imp_uint idx = 3*(y*_width + x);
    return Radiance(_pixel_buffer[idx], _pixel_buffer[idx + 1], _pixel_buffer[idx + 2]);
}

void Image::setRadiance(imp_uint x, imp_uint y, const Radiance& radiance)
{
    imp_uint idx = 3*(y*_width + x);

    _pixel_buffer[idx]     = radiance.r;
    _pixel_buffer[idx + 1] = radiance.g;
    _pixel_buffer[idx + 2] = radiance.b;
}

void Image::accumulateRadiance(imp_uint x, imp_uint y, const Radiance& radiance)
{
    imp_uint idx = 3*(y*_width + x);

    _pixel_buffer[idx]     += radiance.r;
    _pixel_buffer[idx + 1] += radiance.g;
    _pixel_buffer[idx + 2] += radiance.b;
}

void Image::normalizeAccumulatedRadiance(imp_uint x, imp_uint y, imp_float normalization)
{
    imp_uint idx = 3*(y*_width + x);
	
    _pixel_buffer[idx]     *= normalization;
    _pixel_buffer[idx + 1] *= normalization;
    _pixel_buffer[idx + 2] *= normalization;
}

imp_float Image::getDepth(imp_uint x, imp_uint y) const
{
	assert(_depth_buffer);
    return _depth_buffer[y*_width + x];
}

void Image::setDepth(imp_uint x, imp_uint y, imp_float depth)
{
	assert(_depth_buffer);
    _depth_buffer[y*_width + x] = depth;
}

void Image::initializeDepthBuffer(imp_float initial_value)
{
	if (!_depth_buffer)
		_depth_buffer = new imp_float[_width*_height];

	int idx;

    #pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(initial_value) \
                             schedule(static) \
                             if (use_omp)
	for (idx = 0; idx < static_cast<int>(_n_elements); idx++)
	{
		_depth_buffer[idx] = initial_value;
	}
}

void Image::setBackgroundColor(const Color& color)
{
	int idx;

    #pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(color) \
                             schedule(static) \
                             if (use_omp)
    for (idx = 0; idx < 3*static_cast<int>(_n_elements); idx += 3)
    {
        _pixel_buffer[idx]     = color.r;
        _pixel_buffer[idx + 1] = color.g;
        _pixel_buffer[idx + 2] = color.b;
    }
}

void Image::gammaEncode(imp_float normalization)
{
	int idx;
    imp_float inverse_gamma = 1.0f/2.2f;

    #pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(normalization, inverse_gamma) \
                             schedule(static) \
                             if (use_omp)
    for (idx = 0; idx < 3*static_cast<int>(_n_elements); idx++)
    {
        _pixel_buffer[idx] = pow(std::min(1.0f, std::max(0.0f, _pixel_buffer[idx]*normalization)), inverse_gamma);
    }
}

void Image::gammaEncodeApprox(imp_float normalization)
{
	int idx;

    #pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(normalization) \
                             schedule(static) \
                             if (use_omp)
    for (idx = 0; idx < 3*static_cast<int>(_n_elements); idx++)
    {
        _pixel_buffer[idx] = sqrt(std::min(1.0f, std::max(0.0f, _pixel_buffer[idx]*normalization)));
    }
}

Color Image::getApproxGammaEncodedColor(imp_uint x, imp_uint y, imp_float normalization) const
{
    imp_uint idx = 3*(y*_width + x);

	return Color(sqrt(std::min(1.0f, std::max(0.0f, _pixel_buffer[idx    ]*normalization))),
				 sqrt(std::min(1.0f, std::max(0.0f, _pixel_buffer[idx + 1]*normalization))),
				 sqrt(std::min(1.0f, std::max(0.0f, _pixel_buffer[idx + 2]*normalization))));
}

void Image::stretch()
{
    auto min_max_elements = std::minmax_element(_pixel_buffer, _pixel_buffer + _size);
    imp_float min_val = *(min_max_elements.first);
    imp_float max_val = *(min_max_elements.second);
    imp_float range = max_val - min_val;

	int idx;

    assert(range != 0);
	
    #pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(min_val, range) \
                             schedule(static) \
                             if (use_omp)
    for (idx = 0; idx < static_cast<int>(_size); idx++)
    {
        _pixel_buffer[idx] = (_pixel_buffer[idx] - min_val)/range;
    }
}

imp_uint Image::getWidth() const
{
    return _width;
}

imp_uint Image::getHeight() const
{
    return _height;
}

imp_float Image::getAspectRatio() const
{
    return static_cast<imp_float>(_width)/static_cast<imp_float>(_height);
}

const imp_float* Image::getRawPixelArray() const
{
    return _pixel_buffer;
}

void Image::renderDepthMap(bool renormalize)
{
	assert(_depth_buffer);

	imp_float depth_value;
	int idx;

	if (renormalize)
	{
		auto min_max_elements = std::minmax_element(_depth_buffer, _depth_buffer + _n_elements);
		_depth_map_min_val = *(min_max_elements.first);
		imp_float max_val = *(min_max_elements.second);
		
		assert(max_val != _depth_map_min_val);

		_depth_map_norm = 1/(max_val - _depth_map_min_val);
	}

    #pragma omp parallel for default(shared) \
                             private(idx, depth_value) \
                             schedule(static) \
                             if (use_omp)
    for (idx = 0; idx < static_cast<int>(_n_elements); idx++)
    {
		depth_value = (_depth_buffer[idx] - _depth_map_min_val)*_depth_map_norm;
        _pixel_buffer[3*idx] = depth_value;
        _pixel_buffer[3*idx + 1] = depth_value;
        _pixel_buffer[3*idx + 2] = depth_value;
    }
}

void Image::saveAsPPM(const std::string& filename)
{
	_output_buffer = image_util::writePPM(filename, _pixel_buffer, _width, _height, 3, 0, _output_buffer, true);
}

} // Rendering3D
} // Impact
