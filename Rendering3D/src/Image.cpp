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

Image::Image(imp_uint n_cols, imp_uint n_rows)
    : _n_cols(n_cols), _n_rows(n_rows), _n_elements(n_rows*n_cols), _size(3*n_rows*n_cols),
      _pixel_buffer(new imp_float[3*n_rows*n_cols]),
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
        idx = y*_n_cols + x;

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
    imp_uint idx = 3*(y*_n_cols + x);
    return Radiance(_pixel_buffer[idx], _pixel_buffer[idx + 1], _pixel_buffer[idx + 2]);
}

void Image::setRadiance(imp_uint x, imp_uint y, const Radiance& radiance)
{
    imp_uint idx = 3*(y*_n_cols + x);

    _pixel_buffer[idx]     = radiance.r;
    _pixel_buffer[idx + 1] = radiance.g;
    _pixel_buffer[idx + 2] = radiance.b;
}

imp_float Image::getDepth(imp_uint x, imp_uint y) const
{
	assert(_depth_buffer);
    return _depth_buffer[y*_n_cols + x];
}

void Image::setDepth(imp_uint x, imp_uint y, imp_float depth)
{
	assert(_depth_buffer);
    _depth_buffer[y*_n_cols + x] = depth;
}

void Image::initializeDepthBuffer(imp_float initial_value)
{
	if (!_depth_buffer)
		_depth_buffer = new imp_float[_n_cols*_n_rows];

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

void Image::stretch()
{
    auto min_max_elements = std::minmax_element(_pixel_buffer, _pixel_buffer + _size);
    imp_float min_val = *(min_max_elements.first);
    imp_float max_val = *(min_max_elements.second);
    imp_float range = max_val - min_val;

    assert(range != 0);

    for (imp_uint idx = 0; idx < _size; idx++)
    {
        _pixel_buffer[idx] = (_pixel_buffer[idx] - min_val)/range;
    }
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

void Image::writeToOutputBuffer()
{
	if (!_output_buffer)
		_output_buffer = new output_buffer[_size];

	int i, j;
	int offset_in, offset_out;
	int idx_in, idx_out;
	int n_cols = static_cast<int>(_n_cols);
	int n_rows = static_cast<int>(_n_rows);

    #pragma omp parallel for default(shared) \
                             private(i, j, offset_in, offset_out, idx_in, idx_out) \
							 shared(n_rows, n_cols) \
                             schedule(static) \
                             if (use_omp)
	for (i = 0; i < n_rows; i++)
	{
		offset_in = n_cols*i;
		offset_out = n_cols*(n_rows-1 - i);

		for (j = 0; j < n_cols; j++)
		{
			idx_in = 3*(j + offset_in);
			idx_out = 3*(j + offset_out);

			_output_buffer[idx_out]     = static_cast<output_buffer>(255*_pixel_buffer[idx_in]    );
			_output_buffer[idx_out + 1] = static_cast<output_buffer>(255*_pixel_buffer[idx_in + 1]);
			_output_buffer[idx_out + 2] = static_cast<output_buffer>(255*_pixel_buffer[idx_in + 2]);
		}
	}
}

void Image::saveAsPPM(const std::string& filename)
{
	FILE* image_file;

	writeToOutputBuffer();

	fopen_s(&image_file, filename.c_str(), "wb");

	assert(image_file);

	fprintf(image_file, "P6\n");
	fprintf(image_file, "%d %d\n", _n_cols, _n_rows);
	fprintf(image_file, "255\n");
	fwrite(_output_buffer, sizeof(output_buffer), _size, image_file);
	fclose(image_file);
}

} // Rendering3D
} // Impact
