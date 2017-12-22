#include "Texture.hpp"
#include "image_util.hpp"
#include <cassert>
#include <fstream>
#include <algorithm>

namespace Impact {
namespace Rendering3D {

Texture::Texture(const std::string& filename, ValueRange range_type)
	: _pixel_buffer(nullptr),
	  _width(0), _height(0), _n_components(0)
{
	image_util::readPPM(filename, _pixel_buffer, _width, _height, _n_components, range_type);

	_n_pixels = _width*_height;
	_size = _n_pixels*_n_components;
}

Texture::Texture(imp_float* pixel_buffer, imp_uint width, imp_uint height, imp_uint n_components)
	: _pixel_buffer(pixel_buffer),
	  _width(width),
	  _height(height),
	  _n_components(n_components),
	  _n_pixels(width*height),
	  _size(width*height*n_components) {}

Texture::~Texture()
{
	delete _pixel_buffer;
}

void Texture::write(const std::string& filename, ValueRange range_type) const
{
	image_util::writePPM(filename, _pixel_buffer, _width, _height, _n_components, range_type);
}

Color Texture::getColor(const Point2& uv_coord) const
{
	assert(_n_components == 3);

	imp_float u = std::max(0.0f, std::min(1.0f, uv_coord.x));
	imp_float v = std::max(0.0f, std::min(1.0f, uv_coord.y));

	imp_uint left_pixel = static_cast<imp_uint>((_width-1)*u);
	imp_uint lower_pixel = static_cast<imp_uint>((_height-1)*v);

	imp_uint right_pixel = left_pixel + 1;
	imp_uint upper_pixel = lower_pixel + 1;

	if (right_pixel == _width)
		right_pixel = 0;
	if (upper_pixel == _height)
		upper_pixel = 0;

	imp_uint SW_idx = 3*(lower_pixel*_width + left_pixel);
	imp_uint SE_idx = 3*(lower_pixel*_width + right_pixel);
	imp_uint NW_idx = 3*(upper_pixel*_width + left_pixel);
	imp_uint NE_idx = 3*(upper_pixel*_width + right_pixel);

	Color SW_color(_pixel_buffer[SW_idx], _pixel_buffer[SW_idx+1], _pixel_buffer[SW_idx+2]);
	Color SE_color(_pixel_buffer[SE_idx], _pixel_buffer[SE_idx+1], _pixel_buffer[SE_idx+2]);
	Color NW_color(_pixel_buffer[NW_idx], _pixel_buffer[NW_idx+1], _pixel_buffer[NW_idx+2]);
	Color NE_color(_pixel_buffer[NE_idx], _pixel_buffer[NE_idx+1], _pixel_buffer[NE_idx+2]);

	return (SW_color + SE_color + NW_color + NE_color)*0.25f;
}

Color Texture::getNormalDirectionWeights(const Point2& uv_coord) const
{
	return getColor(uv_coord);
}

imp_float Texture::getDisplacementValue(const Point2& uv_coord) const
{
	assert(_n_components == 1);

	imp_float u = std::max(0.0f, std::min(1.0f, uv_coord.x));
	imp_float v = std::max(0.0f, std::min(1.0f, uv_coord.y));

	imp_uint left_pixel = static_cast<imp_uint>((_width-1)*u);
	imp_uint lower_pixel = static_cast<imp_uint>((_height-1)*v);

	imp_uint right_pixel = left_pixel + 1;
	imp_uint upper_pixel = lower_pixel + 1;

	if (right_pixel == _width)
		right_pixel = 0;
	if (upper_pixel == _height)
		upper_pixel = 0;

	imp_uint SW_idx = (lower_pixel*_width + left_pixel);
	imp_uint SE_idx = (lower_pixel*_width + right_pixel);
	imp_uint NW_idx = (upper_pixel*_width + left_pixel);
	imp_uint NE_idx = (upper_pixel*_width + right_pixel);

	return (_pixel_buffer[SW_idx] +
		    _pixel_buffer[SE_idx] +
			_pixel_buffer[NW_idx] +
			_pixel_buffer[NE_idx])*0.25f;
}

imp_uint Texture::getWidth() const
{
    return _width;
}

imp_uint Texture::getHeight() const
{
    return _height;
}

imp_uint Texture::getNumberOfComponents() const
{
    return _n_components;
}

const imp_float* Texture::getRawPixelArray() const
{
    return _pixel_buffer;
}

} // Rendering3D
} // Impact
