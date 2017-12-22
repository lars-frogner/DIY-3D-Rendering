#pragma once
#include "precision.hpp"
#include "Texture.hpp"
#include <string>

namespace Impact {
namespace Rendering3D {

class TextureSynthesizer {

private:

protected:
	inline static void computeNormalVector(imp_float* normal_map,
										   imp_uint idx,
										   imp_float dz_du,
										   imp_float dz_dv);

public:
	static Texture* normalMapFromDisplacementMap(const Texture* displacement_map,
												 imp_float displacement_scale,
												 bool periodic_in_u = false,
												 bool periodic_in_v = false);

	static void normalMapFromDisplacementMap(const std::string& displacement_map_filename,
											 imp_float displacement_scale,
											 bool periodic_in_u = false,
											 bool periodic_in_v = false,
											 const std::string& normal_map_filename = "");
};

} // Rendering3D
} // Impact
