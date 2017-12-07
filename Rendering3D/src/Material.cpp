#include "Material.hpp"
#include <cmath>

namespace Impact {
namespace Rendering3D {

Reflectance Material::getFresnelReflectance(const Color& normal_incidence_reflection, imp_float cos_incoming_angle)
{
	// Schlick approximation

	imp_float one_minus_cos_theta_i = 1 - cos_incoming_angle;

	return normal_incidence_reflection + (1 - normal_incidence_reflection)*(one_minus_cos_theta_i*one_minus_cos_theta_i*one_minus_cos_theta_i*one_minus_cos_theta_i*one_minus_cos_theta_i);
}

} // Rendering3D
} // Impact
