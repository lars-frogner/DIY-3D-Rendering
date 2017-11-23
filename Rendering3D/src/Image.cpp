#include "Image.hpp"
#include <omp.h>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace Impact {
namespace Rendering3D {

Image::Image(imp_uint n_cols, imp_uint n_rows)
    : _n_cols(n_cols), _n_rows(n_rows), _size(n_rows*n_cols),
      _pixel_values(3*n_rows*n_cols, 0),
	  _depth_buffer(n_rows*n_cols, -1) {}

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
        idx = y*_n_cols + x;
        assert(idx < _size);
        idx *= 3;
        _pixel_values[idx] = luminance;
        _pixel_values[idx + 1] = luminance;
        _pixel_values[idx + 2] = luminance;

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

void Image::drawTriangle(const Vertex3& v0, const Vertex3& v1, const Vertex3& v2)
{
    imp_int x0_i = static_cast<imp_int>(round(v0.x));
    imp_int y0_i = static_cast<imp_int>(round(v0.y));
    imp_int x1_i = static_cast<imp_int>(round(v1.x));
    imp_int y1_i = static_cast<imp_int>(round(v1.y));
    imp_int x2_i = static_cast<imp_int>(round(v2.x));
    imp_int y2_i = static_cast<imp_int>(round(v2.y));

    imp_int x_min = std::min(x0_i, std::min(x1_i, x2_i));
    imp_int x_max = std::max(x0_i, std::max(x1_i, x2_i));
    imp_int y_min = std::min(y0_i, std::min(y1_i, y2_i));
    imp_int y_max = std::max(y0_i, std::max(y1_i, y2_i));

	if (x_min < 0 || x_max >= static_cast<imp_int>(_n_cols) || y_min < 0 || y_max >= static_cast<imp_int>(_n_rows))
		return;

    //assert(x_min >= 0 && x_max < _n_cols);
    //assert(y_min >= 0 && y_max < _n_rows);

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
                idx = y*_n_cols + x;
                z = alpha*v0.z + beta*v1.z + gamma*v2.z;

                if (z > _depth_buffer[idx])
                {
                    _depth_buffer[idx] = z;

                    idx *= 3;
                    _pixel_values[idx] = alpha*v0.color.r + beta*v1.color.r + gamma*v2.color.r;
                    _pixel_values[idx + 1] = alpha*v0.color.g + beta*v1.color.g + gamma*v2.color.g;
                    _pixel_values[idx + 2] = alpha*v0.color.b + beta*v1.color.b + gamma*v2.color.b;
                }
            }
        }
    }
}

Radiance Image::getRadiance(imp_uint x, imp_uint y) const
{
    imp_uint idx = 3*(y*_n_cols + x);
    return Radiance(_pixel_values[idx], _pixel_values[idx + 1], _pixel_values[idx + 2]);
}

Image& Image::setRadiance(imp_uint x, imp_uint y, const Radiance& radiance)
{
    imp_uint idx = 3*(y*_n_cols + x);

    _pixel_values[idx]     = radiance.r;
    _pixel_values[idx + 1] = radiance.g;
    _pixel_values[idx + 2] = radiance.b;

    return *this;
}

imp_float Image::getDepth(imp_uint x, imp_uint y) const
{
    return _depth_buffer[y*_n_cols + x];
}

Image& Image::setDepth(imp_uint x, imp_uint y, imp_float depth)
{
    _depth_buffer[y*_n_cols + x] = depth;
    return *this;
}

Image& Image::initializeDepthBuffer(imp_float initial_value)
{
	int idx;

    #pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(initial_value) \
                             schedule(static) \
                             if (use_omp)
	for (idx = 0; idx < static_cast<int>(_size); idx++)
	{
		_depth_buffer[idx] = initial_value;
	}

    return *this;
}

Image& Image::setBackgroundColor(const Color& color)
{
	int idx;

    #pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(color) \
                             schedule(static) \
                             if (use_omp)
    for (idx = 0; idx < 3*static_cast<int>(_size); idx += 3)
    {
        _pixel_values[idx]     = color.r;
        _pixel_values[idx + 1] = color.g;
        _pixel_values[idx + 2] = color.b;
    }

    return *this;
}

Image& Image::gammaEncode(imp_float normalization)
{
	int idx;
    imp_float inverse_gamma = 1.0f/2.2f;

    #pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(normalization, inverse_gamma) \
                             schedule(static) \
                             if (use_omp)
    for (idx = 0; idx < 3*static_cast<int>(_size); idx++)
    {
        _pixel_values[idx] = pow(std::min(1.0f, std::max(0.0f, _pixel_values[idx]*normalization)), inverse_gamma);
    }

    return *this;
}

Image& Image::gammaEncodeApprox(imp_float normalization)
{
	int idx;

    #pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(normalization) \
                             schedule(static) \
                             if (use_omp)
    for (idx = 0; idx < 3*static_cast<int>(_size); idx++)
    {
        _pixel_values[idx] = sqrt(std::min(1.0f, std::max(0.0f, _pixel_values[idx]*normalization)));
    }

    return *this;
}

Image& Image::stretch()
{
    auto min_max_elements = std::minmax_element(_pixel_values.begin(), _pixel_values.end());
    imp_float min_val = *(min_max_elements.first);
    imp_float max_val = *(min_max_elements.second);
    imp_float range = max_val - min_val;

    assert(range != 0);

    for (imp_uint idx = 0; idx < 3*_size; idx++)
    {
        _pixel_values[idx] = (_pixel_values[idx] - min_val)/range;
    }

    return *this;
}

imp_uint Image::getWidth() const
{
    return _n_cols;
}

imp_uint Image::getHeight() const
{
    return _n_rows;
}

imp_float Image::getAspectRatio() const
{
    return static_cast<imp_float>(_n_cols)/static_cast<imp_float>(_n_rows);
}

const float* Image::getRawPixelArray() const
{
    return _pixel_values.data();
}

} // Rendering3D
} // Impact
