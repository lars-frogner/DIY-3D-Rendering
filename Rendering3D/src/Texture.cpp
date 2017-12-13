#include "Texture.hpp"
#include "string_util.hpp"
#include <cassert>
#include <fstream>
#include <iostream>

namespace Impact {
namespace Rendering3D {

Texture::Texture(const std::string& filename)
{
	readPPM(filename);
}

Texture::~Texture()
{
	delete _pixel_buffer;
}

void Texture::readPPM(const std::string& filename)
{
	std::string format_identifier;
	std::string dimension_string;
	std::string max_color_value_string;

	std::ifstream file;
    file.open(filename.c_str(), std::ios::binary);
	
	assert(file.is_open());

	// Read format identifier
	std::getline(file, format_identifier);
	string_util::trim(format_identifier);

	assert(format_identifier == "P5" || format_identifier == "P6");

	if (format_identifier == "P5")
	{
		_n_components = 1;
	}
	else if (format_identifier == "P6")
	{
		_n_components = 3;
	}

	// Read image dimensions
	std::getline(file, dimension_string);
	string_util::trim(dimension_string);

	const std::vector<std::string>& splitted = string_util::split(dimension_string);

	assert(splitted.size() == 1 || splitted.size() == 2);

	if (splitted.size() == 1)
	{
		_width = static_cast<imp_uint>(atoi(dimension_string.c_str()));

		std::getline(file, dimension_string);
		string_util::trim(dimension_string);
		
		_height = static_cast<imp_uint>(atoi(dimension_string.c_str()));
	}
	else if (splitted.size() == 2)
	{
		_width = static_cast<imp_uint>(atoi(splitted[0].c_str()));
		_height = static_cast<imp_uint>(atoi(splitted[1].c_str()));
	}

	// Make sure dimensions are powers of 2
	assert((_width & (_width - 1)) == 0);
	assert((_height & (_height - 1)) == 0);

	_n_pixels = _width*_height;
	_size = _n_components*_n_pixels;
	
	// Read color scale
	std::getline(file, max_color_value_string);
	string_util::trim(max_color_value_string);
	
    imp_uint max_color_value = static_cast<imp_uint>(atoi(max_color_value_string.c_str()));

	assert(max_color_value > 0 && max_color_value < 256);
	
	imp_float color_norm = 1.0f/max_color_value;

	// Read pixels
	typedef unsigned char sample_type;
	sample_type* input_buffer = new sample_type[_size];
    file.read(reinterpret_cast<char*>(input_buffer), _size*sizeof(sample_type));
	file.close();

	// Convert to normalized format

	_pixel_buffer = new imp_float[_size];

	int i, j;
	int offset_in, offset_out;
	int idx_in, idx_out;
	int width = static_cast<int>(_width);
	int height = static_cast<int>(_height);

	if (_n_components == 1)
	{
		#pragma omp parallel for default(shared) \
								 private(i, j, offset_in, offset_out, idx_in, idx_out) \
								 shared(width, height) \
								 schedule(static)
		for (i = 0; i < height; i++)
		{
			offset_in = width*i;
			offset_out = width*(height-1 - i);

			for (j = 0; j < width; j++)
			{
				idx_in = j + offset_in;
				idx_out = j + offset_out;

				_pixel_buffer[idx_in] = color_norm*static_cast<imp_float>(input_buffer[idx_out]);
			}
		}
	}
	else
	{
		#pragma omp parallel for default(shared) \
								 private(i, j, offset_in, offset_out, idx_in, idx_out) \
								 shared(width, height) \
								 schedule(static)
		for (i = 0; i < height; i++)
		{
			offset_in = width*i;
			offset_out = width*(height-1 - i);

			for (j = 0; j < width; j++)
			{
				idx_in = 3*(j + offset_in);
				idx_out = 3*(j + offset_out);

				_pixel_buffer[idx_in]     = color_norm*static_cast<imp_float>(input_buffer[idx_out]    );
				_pixel_buffer[idx_in + 1] = color_norm*static_cast<imp_float>(input_buffer[idx_out + 1]);
				_pixel_buffer[idx_in + 2] = color_norm*static_cast<imp_float>(input_buffer[idx_out + 2]);
			}
		}
	}

	delete input_buffer;
}

Color Texture::getColor(const Point2& uv_coord) const
{
	assert(_n_components == 3);
	assert(uv_coord.x >= 0.0f && uv_coord.x <= 1.0f);
	assert(uv_coord.y >= 0.0f && uv_coord.y <= 1.0f);

	imp_uint left_pixel = static_cast<imp_uint>((_width-1)*uv_coord.x);
	imp_uint lower_pixel = static_cast<imp_uint>((_height-1)*uv_coord.y);

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

imp_uint Texture::getWidth() const
{
    return _width;
}

imp_uint Texture::getHeight() const
{
    return _height;
}

const imp_float* Texture::getRawPixelArray() const
{
    return _pixel_buffer;
}

} // Rendering3D
} // Impact
