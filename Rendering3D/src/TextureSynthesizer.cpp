#include "TextureSynthesizer.hpp"
#include "string_util.hpp"
#include <cassert>
#include <vector>
#include <iostream>

namespace Impact {
namespace Rendering3D {

inline void TextureSynthesizer::computeNormalVector(imp_float* normal_map,
													imp_uint idx,
													imp_float dz_du,
													imp_float dz_dv)
{
	imp_float norm = 1.0f/sqrt(dz_du*dz_du + dz_dv*dz_dv + 1.0f);

	normal_map[3*idx    ] = -dz_du*norm;
	normal_map[3*idx + 1] = -dz_dv*norm;
	normal_map[3*idx + 2] = norm;
}

Texture* TextureSynthesizer::normalMapFromDisplacementMap(const Texture* displacement_map_texture,
														  imp_float displacement_scale,
														  bool periodic_in_u /* = false */,
														  bool periodic_in_v /* = false */)
{
	assert(displacement_map_texture->getNumberOfComponents() == 1);

	const imp_float* displacement_map = displacement_map_texture->getRawPixelArray();

	// Compute normal map

	imp_uint width = displacement_map_texture->getWidth();
	imp_uint height = displacement_map_texture->getHeight();

	imp_float* normal_map = new imp_float[width*height*3];

	imp_uint i, j, idx;
	imp_float dz_i, dz_j;
	imp_float u_scale = displacement_scale*width, v_scale = displacement_scale*height;
	
	// i = 0, j = 0
	idx = 0 + 0*width;

	dz_i = (periodic_in_u)? (displacement_map[idx +     1] - displacement_map[idx + (width-1)       ])*0.5f : displacement_map[idx +     1] - displacement_map[idx];
	dz_j = (periodic_in_v)? (displacement_map[idx + width] - displacement_map[idx + (height-1)*width])*0.5f : displacement_map[idx + width] - displacement_map[idx];

	computeNormalVector(normal_map, idx, dz_i*u_scale, dz_j*v_scale);

	// i in [1, width-1], j = 0
	for (i = 1; i < width-1; i++)
	{
		idx = i + 0*width;

		dz_i = (displacement_map[idx + 1] - displacement_map[idx - 1])*0.5f;
		dz_j = (periodic_in_v)? (displacement_map[idx + width] - displacement_map[idx + (height-1)*width])*0.5f : displacement_map[idx + width] - displacement_map[idx];
		
		computeNormalVector(normal_map, idx, dz_i*u_scale, dz_j*v_scale);
	}
		
	// i = width-1, j = 0
	idx = width-1 + 0*width;

	dz_i = (periodic_in_u)? (displacement_map[idx - (width-1)] - displacement_map[idx -                1])*0.5f : displacement_map[idx        ] - displacement_map[idx - 1];
	dz_j = (periodic_in_v)? (displacement_map[idx +     width] - displacement_map[idx + (height-1)*width])*0.5f : displacement_map[idx + width] - displacement_map[idx    ];
	
	computeNormalVector(normal_map, idx, dz_i*u_scale, dz_j*v_scale);

	// j in [1, height-1]
	for (j = 1; j < height-1; j++)
	{
		// i = 0
		idx = 0 + j*width;

		dz_i = (periodic_in_u)? (displacement_map[idx + 1] - displacement_map[idx + (width-1)])*0.5f : displacement_map[idx + 1] - displacement_map[idx];
		dz_j = (displacement_map[idx + width] - displacement_map[idx - width])*0.5f;
		
		computeNormalVector(normal_map, idx, dz_i*u_scale, dz_j*v_scale);

		// i in [1, width-1]
		for (i = 1; i < width-1; i++)
		{
			idx = i + j*width;

			dz_i = (displacement_map[idx +     1] - displacement_map[idx -     1])*0.5f;
			dz_j = (displacement_map[idx + width] - displacement_map[idx - width])*0.5f;
			
			computeNormalVector(normal_map, idx, dz_i*u_scale, dz_j*v_scale);
		}
		
		// i = width-1
		idx = width-1 + j*width;

		dz_i = (periodic_in_u)? (displacement_map[idx - (width-1)] - displacement_map[idx - 1])*0.5f : displacement_map[idx] - displacement_map[idx - 1];
		dz_j = (displacement_map[idx + width] - displacement_map[idx - width])*0.5f;
		
		computeNormalVector(normal_map, idx, dz_i*u_scale, dz_j*v_scale);
	}
	
	// i = 0, j = height-1
	idx = 0 + (height-1)*width;

	dz_i = (periodic_in_u)? (displacement_map[idx +                1] - displacement_map[idx + (width-1)])*0.5f : displacement_map[idx + 1] - displacement_map[idx        ];
	dz_j = (periodic_in_v)? (displacement_map[idx - (height-1)*width] - displacement_map[idx -     width])*0.5f : displacement_map[idx    ] - displacement_map[idx - width];
	
	computeNormalVector(normal_map, idx, dz_i*u_scale, dz_j*v_scale);
	
	// i in [1, width-1], j = height-1
	for (i = 1; i < width-1; i++)
	{
		idx = i + (height-1)*width;

		dz_i = (displacement_map[idx + 1] - displacement_map[idx - 1])*0.5f;
		dz_j = (periodic_in_v)? (displacement_map[idx - (height-1)*width] - displacement_map[idx - width])*0.5f : displacement_map[idx] - displacement_map[idx - width];
		
		computeNormalVector(normal_map, idx, dz_i*u_scale, dz_j*v_scale);
	}
		
	// i = width-1, j = height-1
	idx = width-1 + (height-1)*width;

	dz_i = (periodic_in_u)? (displacement_map[idx - (width-1)       ] - displacement_map[idx -     1])*0.5f : displacement_map[idx] - displacement_map[idx -     1];
	dz_j = (periodic_in_v)? (displacement_map[idx - (height-1)*width] - displacement_map[idx - width])*0.5f : displacement_map[idx] - displacement_map[idx - width];
	
	computeNormalVector(normal_map, idx, dz_i*u_scale, dz_j*v_scale);

	return new Texture(normal_map, width, height, 3);
}

void TextureSynthesizer::normalMapFromDisplacementMap(const std::string& displacement_map_filename,
													  imp_float displacement_scale,
													  bool periodic_in_u /* = false */,
													  bool periodic_in_v /* = false */,
													  const std::string& normal_map_filename /* = "" */)
{
	Texture* displacement_map_texture = new Texture(displacement_map_filename, Texture::ValueRange::DISPLACEMENT_MAP);

	Texture* normal_map_texture = normalMapFromDisplacementMap(displacement_map_texture, displacement_scale, periodic_in_u, periodic_in_v);

	delete displacement_map_texture;

	std::string normal_map_output_filename = normal_map_filename;
	
	if (normal_map_output_filename.size() == 0)
		normal_map_output_filename = string_util::join(string_util::split(displacement_map_filename, '.'), ".", 0, -2) + "_normal_map.ppm";
	
	normal_map_texture->write(normal_map_output_filename, Texture::ValueRange::NORMAL_MAP);

	delete normal_map_texture;
}

} // Rendering3D
} // Impact
