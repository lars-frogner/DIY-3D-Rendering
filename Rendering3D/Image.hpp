#pragma once
#include <math.h>
#include <algorithm>
#include <limits>
#include <iostream>
#include "../Geometry2D/Vertex.hpp"
#include "Color.hpp"

namespace Rendering3D {

template <typename F>
class Image {

private:
    typedef Geometry3D::Vertex<F> Vertex;

    size_t _n_cols, _n_rows, _size;
    std::vector<float> _pixel_values;
    std::vector<F> _depth_buffer;

    const F _eps_barycentric = -1.e-7f;

public:
    
    Image<F>(size_t n_cols, size_t n_rows);

    size_t getWidth() const;
    size_t getHeight() const;
    F getAspectRatio() const;
    const float* getRawPixelArray() const;
    
    Radiance getRadiance(size_t x, size_t y) const;
    Image<F>& setRadiance(size_t x, size_t y, const Radiance& radiance);
    
    F getDepth(size_t x, size_t y) const;
    Image<F>& setDepth(size_t x, size_t y, F depth);
    
    Image<F>& initializeDepthBuffer(F initial_value);
    Image<F>& setBackgroundColor(const Color& color);

    Image<F>& gammaEncode(float normalization);
    Image<F>& stretch();

    void drawLine(F x0, F y0, F x1, F y1, float luminance);
    void drawTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2);
};

template <typename F>
Image<F>::Image(size_t n_cols, size_t n_rows)
    : _n_cols(n_cols), _n_rows(n_rows), _size(n_rows*n_cols),
      _pixel_values(3*n_rows*n_cols, 0) {}

template <typename F>
void Image<F>::drawLine(F x0, F y0, F x1, F y1, float luminance)
{
    // Bresenham's line algorithm

    size_t idx;

    int x = static_cast<int>(round(x0));
    int y = static_cast<int>(round(y0));

    int w = static_cast<int>(round(x1)) - x;
    int h = static_cast<int>(round(y1)) - y;

    int dx0 = 0, dy0 = 0, dx1 = 0, dy1 = 0;

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

    int longest = abs(w);
    int shortest = abs(h);
    int temp;

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

    for (int i = 0; i <= longest; i++)
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

template <typename F>
void Image<F>::drawTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2)
{
    int x0_i = static_cast<int>(round(v0.x));
    int y0_i = static_cast<int>(round(v0.y));
    int x1_i = static_cast<int>(round(v1.x));
    int y1_i = static_cast<int>(round(v1.y));
    int x2_i = static_cast<int>(round(v2.x));
    int y2_i = static_cast<int>(round(v2.y));

    int x_min = std::min(x0_i, std::min(x1_i, x2_i));
    int x_max = std::max(x0_i, std::max(x1_i, x2_i));
    int y_min = std::min(y0_i, std::min(y1_i, y2_i));
    int y_max = std::max(y0_i, std::max(y1_i, y2_i));

    assert(x_min >= 0 && x_max < _n_cols);
    assert(y_min >= 0 && y_max < _n_rows);

    int dx1 = x1_i - x0_i, dy1 = y1_i - y0_i;
    int dx2 = x2_i - x0_i, dy2 = y2_i - y0_i;
    int dxp, dyp;
    
    F factor;
    F alpha, beta, gamma;
    F z;

    int x, y;
    size_t idx;

    for (y = y_min; y <= y_max; y++) {
        for (x = x_min; x <= x_max; x++)
        {
            dxp = x - x0_i; dyp = y - y0_i;

            factor = 1/static_cast<F>(dx1*dy2 - dx2*dy1);

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

template <typename F>
Radiance Image<F>::getRadiance(size_t x, size_t y) const
{
    size_t idx = 3*(y*_n_cols + x);
    return Radiance(_pixel_values[idx], _pixel_values[idx + 1], _pixel_values[idx + 2]);
}

template <typename F>
Image<F>& Image<F>::setRadiance(size_t x, size_t y, const Radiance& radiance)
{
    size_t idx = 3*(y*_n_cols + x);

    _pixel_values[idx]     = radiance.r;
    _pixel_values[idx + 1] = radiance.g;
    _pixel_values[idx + 2] = radiance.b;

    return *this;
}

template <typename F>
F Image<F>::getDepth(size_t x, size_t y) const
{
    return _depth_buffer[y*_n_cols + x];
}

template <typename F>
Image<F>& Image<F>::setDepth(size_t x, size_t y, F depth)
{
    _depth_buffer[y*_n_cols + x] = depth;
    return *this;
}

template <typename F>
Image<F>& Image<F>::initializeDepthBuffer(F initial_value)
{
    _depth_buffer.resize(_size, initial_value);
    return *this;
}

template <typename F>
Image<F>& Image<F>::setBackgroundColor(const Color& color)
{
    for (size_t idx = 0; idx < 3*_size; idx += 3)
    {
        _pixel_values[idx]     = color.r;
        _pixel_values[idx + 1] = color.g;
        _pixel_values[idx + 2] = color.b;
    }
    return *this;
}

template <typename F>
Image<F>& Image<F>::gammaEncode(float normalization)
{
    float inverse_gamma = 1.0f/2.2f;
    for (size_t idx = 0; idx < 3*_size; idx++)
    {
        _pixel_values[idx] = pow(std::min(1.0f, std::max(0.0f, _pixel_values[idx]*normalization)), inverse_gamma);
    }

    return *this;
}

template <typename F>
Image<F>& Image<F>::stretch()
{
    auto min_max_elements = std::minmax_element(_pixel_values.begin(), _pixel_values.end());
    float min_val = *(min_max_elements.first);
    float max_val = *(min_max_elements.second);
    float range = max_val - min_val;

    assert(range != 0);

    for (size_t idx = 0; idx < 3*_size; idx++)
    {
        _pixel_values[idx] = (_pixel_values[idx] - min_val)/range;
    }

    return *this;
}

template <typename F>
size_t Image<F>::getWidth() const
{
    return _n_cols;
}

template <typename F>
size_t Image<F>::getHeight() const
{
    return _n_rows;
}

template <typename F>
F Image<F>::getAspectRatio() const
{
    return static_cast<F>(_n_cols)/static_cast<F>(_n_rows);
}

template <typename F>
const float* Image<F>::getRawPixelArray() const
{
    return _pixel_values.data();
}

} // Rendering3D
