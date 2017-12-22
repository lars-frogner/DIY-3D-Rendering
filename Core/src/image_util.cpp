#include "image_util.hpp"
#include "string_util.hpp"
#include <cassert>
#include <fstream>
#include <iostream>

namespace Impact {
namespace image_util {

imp_ppm_data* readPPM(const std::string& filename,
					  imp_float*& data,
					  imp_uint& width, imp_uint& height, imp_uint& n_components,
					  imp_uint range_type,
					  bool is_texture /* = true */,
					  imp_ppm_data* input_buffer /* = nullptr */,
					  bool keep_input_buffer /* = false */)
{
	std::ifstream file;
	std::string line;
	imp_uint added_header_entries = 0;
	imp_uint n_header_entries = added_header_entries;
	std::vector<std::string> header_entries(4);

	imp_uint current_size = (data || input_buffer)? width*height*n_components : 0;

	// Open file for input
    file.open(filename.c_str(), std::ios::binary);
	
	assert(file.is_open());

	// Read header entries
	while (true)
	{
		std::getline(file, line);
		const std::vector<std::string>& splitted = string_util::split(line);

		n_header_entries += static_cast<imp_uint>(splitted.size());

		assert(n_header_entries <= 4);

		header_entries.insert(header_entries.begin() + added_header_entries, splitted.begin(), splitted.end());

		added_header_entries = n_header_entries;

		if (added_header_entries == 4)
			break;
	}

	// Read format identifier
	assert(header_entries[0] == "P5" || header_entries[0] == "P6");

	if (header_entries[0] == "P5")
	{
		n_components = 1;
	}
	else if (header_entries[0] == "P6")
	{
		n_components = 3;
	}

	// Read image dimensions
	width = static_cast<imp_uint>(atoi(header_entries[1].c_str()));
	height = static_cast<imp_uint>(atoi(header_entries[2].c_str()));

	// Make sure dimensions are powers of 2
	assert(!is_texture || (width & (width - 1)) == 0);
	assert(!is_texture || (height & (height - 1)) == 0);
	
	// Read color scale
    imp_uint max_color_value = static_cast<imp_uint>(atoi(header_entries[3].c_str()));
	assert(max_color_value == 255);

	imp_int color_offset;
	imp_float color_norm;

	assert(range_type == 0 || range_type == 1);

	if (max_color_value == 255) // Values are stored as unsigned bytes with range [0, 255]
	{
		if (range_type == 0) // Scale to range [0, 1]
		{
			color_offset = 0;
		}
		else if (range_type == 1) // Scale to range (-1, 1], with 0 corresponding to byte value 127
		{
			color_offset = -127;
		}

		color_norm = 1.0f/(255 + color_offset);
	}
	else // Values are stored as signed bytes with range [-128, 127]
	{
		if (range_type == 0) // Scale to range [0, 1]
		{
			color_offset = 128;
		}
		else if (range_type == 1) // Scale to range (-1, 1], with 0 corresponding to byte value 127
		{
			color_offset = 1;
		}
			
		color_norm = 1.0f/(127 + color_offset);
	}

	// Read pixels

	imp_uint n_pixels = width*height;
	imp_uint size = n_components*n_pixels;

	if (!(input_buffer && current_size == size))
		input_buffer = new imp_ppm_data[size];

    file.read(reinterpret_cast<char*>(input_buffer), size*sizeof(imp_ppm_data));
	file.close();

	// Convert to normalized format

	if (!(data && current_size == size))
		data = new imp_float[size];

	int i, j;
	int offset_in, offset_out;
	int idx_in, idx_out;
	int width_i = static_cast<int>(width);
	int height_i = static_cast<int>(height);

	if (n_components == 1)
	{
		#pragma omp parallel for default(shared) \
								 private(i, j, offset_in, offset_out, idx_in, idx_out) \
								 shared(width_i, height_i, color_norm, data, input_buffer) \
								 schedule(static)
		for (i = 0; i < height_i; i++)
		{
			offset_in = width_i*i;
			offset_out = width_i*(height_i-1 - i);

			for (j = 0; j < width_i; j++)
			{
				idx_in = j + offset_in;
				idx_out = j + offset_out;

				data[idx_in] = color_norm*(color_offset + static_cast<imp_int>(input_buffer[idx_out]));
			}
		}
	}
	else
	{
		#pragma omp parallel for default(shared) \
								 private(i, j, offset_in, offset_out, idx_in, idx_out) \
								 shared(width_i, height_i, color_norm, data, input_buffer) \
								 schedule(static)
		for (i = 0; i < height_i; i++)
		{
			offset_in = width_i*i;
			offset_out = width_i*(height_i-1 - i);

			for (j = 0; j < width_i; j++)
			{
				idx_in = 3*(j + offset_in);
				idx_out = 3*(j + offset_out);

				data[idx_in]     = color_norm*(color_offset + static_cast<imp_int>(input_buffer[idx_out]    ));
				data[idx_in + 1] = color_norm*(color_offset + static_cast<imp_int>(input_buffer[idx_out + 1]));
				data[idx_in + 2] = color_norm*(color_offset + static_cast<imp_int>(input_buffer[idx_out + 2]));
			}
		}
	}

	if (!keep_input_buffer)
		delete input_buffer;

	return input_buffer;
}

imp_ppm_data* writePPM(const std::string& filename,
					   const imp_float* data,
					   imp_uint width, imp_uint height, imp_uint n_components,
					   imp_uint range_type,
					   imp_ppm_data* output_buffer /* = nullptr */,
					   bool keep_output_buffer /* = false */)
{
	assert(data);
	assert(n_components == 1 || n_components == 3);

	imp_uint size = width*height*n_components;

	imp_float color_offset, color_norm;

	assert(range_type == 0 || range_type == 1);

	if (range_type == 0) // Scale from range [0, 1]
	{
		color_offset = 0.0f;
	}
	else if (range_type == 1) // Scale from range (-1, 1], with 0 corresponding to byte value 127
	{
		color_offset = 127.0f;
	}

	color_norm = 255.0f - color_offset;

	// Convert to PPM format

	if (!output_buffer)
		output_buffer = new imp_ppm_data[size];

	int i, j;
	int offset_in, offset_out;
	int idx_in, idx_out;
	int width_i = static_cast<int>(width);
	int height_i = static_cast<int>(height);

	if (n_components == 1)
	{
		#pragma omp parallel for default(shared) \
								 private(i, j, offset_in, offset_out, idx_in, idx_out) \
								 shared(width_i, height_i, data, output_buffer) \
								 schedule(static)
		for (i = 0; i < height_i; i++)
		{
			offset_in = width_i*i;
			offset_out = width_i*(height_i-1 - i);

			for (j = 0; j < width_i; j++)
			{
				idx_in = j + offset_in;
				idx_out = j + offset_out;

				output_buffer[idx_out] = static_cast<imp_ppm_data>(color_offset + color_norm*data[idx_in]);
			}
		}
	}
	else
	{
		#pragma omp parallel for default(shared) \
								 private(i, j, offset_in, offset_out, idx_in, idx_out) \
								 shared(width_i, height_i, data, output_buffer) \
								 schedule(static)
		for (i = 0; i < height_i; i++)
		{
			offset_in = width_i*i;
			offset_out = width_i*(height_i-1 - i);

			for (j = 0; j < width_i; j++)
			{
				idx_in = 3*(j + offset_in);
				idx_out = 3*(j + offset_out);

				output_buffer[idx_out]     = static_cast<imp_ppm_data>(color_offset + color_norm*data[idx_in]    );
				output_buffer[idx_out + 1] = static_cast<imp_ppm_data>(color_offset + color_norm*data[idx_in + 1]);
				output_buffer[idx_out + 2] = static_cast<imp_ppm_data>(color_offset + color_norm*data[idx_in + 2]);
			}
		}
	}
	
	// Open file for output
	std::ofstream file;
    file.open(filename.c_str(), std::ios::binary);

	assert(file.is_open());
	
	// Write header
	file << ((n_components == 1)? "P5" : "P6") << std::endl;
	file << width << " " << height << std::endl;
	file << 255 << std::endl;

	// Write data
    file.write(reinterpret_cast<char*>(output_buffer), size*sizeof(imp_ppm_data));
	file.close();

	if (!keep_output_buffer)
		delete output_buffer;

	return output_buffer;
}

} // image_util
} // Impact