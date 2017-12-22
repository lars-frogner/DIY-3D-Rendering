#pragma once
#include "precision.hpp"
#include <string>

namespace Impact {
namespace image_util {

typedef unsigned char imp_ppm_data;

imp_ppm_data* readPPM(const std::string& filename,
					  imp_float*& data,
					  imp_uint& width, imp_uint& height, imp_uint& n_components,
					  imp_uint range_type,
					  bool is_texture = true,
					  imp_ppm_data* input_buffer = nullptr,
					  bool keep_input_buffer = false);

imp_ppm_data* writePPM(const std::string& filename,
					   const imp_float* data,
					   imp_uint width, imp_uint height, imp_uint n_components,
					   imp_uint range_type,
					   imp_ppm_data* output_buffer = nullptr,
					   bool keep_output_buffer = false);

} // image_util
} // Impact