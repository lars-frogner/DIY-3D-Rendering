#include "BlinnPhongMaterial.hpp"
#include "math_util.hpp"
#include "LinearTransformation.hpp"
#include <cmath>
#include <algorithm>

namespace Impact {
namespace Rendering3D {

BlinnPhongMaterial::BlinnPhongMaterial()
    : _lambertian_reflectance(Color::black()),
      _refractive_index(Color::white()),
      _coefficient_of_extinction(Color::black()),
      _transmittance(Color::black()),
      _attenuation(Color::black()),
      _glossy_exponent(0)
{
    _initialize();
}

BlinnPhongMaterial::BlinnPhongMaterial(const Reflectance& new_lambertian_reflectance,
									   const Color& new_glossy_reflectance,
									   imp_float new_glossy_exponent)
    : _lambertian_reflectance(new_lambertian_reflectance),
      _glossy_reflectance(new_glossy_reflectance),
      _glossy_exponent(new_glossy_exponent),
      _transmittance(Color::black()),
      _attenuation(Color::black())
{
    _convert_glossy_reflectance_to_refractive_index();
    _initialize();
}

BlinnPhongMaterial::BlinnPhongMaterial(const Reflectance& new_lambertian_reflectance,
									   const Color& new_refractive_index,
									   const Color& new_coefficient_of_extinction,
									   const Color& new_transmittance,
									   const Color& new_attenuation,
									   imp_float new_glossy_exponent)
    : _lambertian_reflectance(new_lambertian_reflectance),
      _refractive_index(new_refractive_index),
      _coefficient_of_extinction(new_coefficient_of_extinction),
      _transmittance(new_transmittance),
      _attenuation(new_attenuation),
	  _glossy_exponent(new_glossy_exponent)
{
    _initialize();
}

BlinnPhongMaterial::BlinnPhongMaterial(const BlinnPhongMaterial& other)
    : _lambertian_reflectance(other._lambertian_reflectance),
      _refractive_index(other._refractive_index),
      _coefficient_of_extinction(other._coefficient_of_extinction),
      _transmittance(other._transmittance),
      _attenuation(other._attenuation),
      _glossy_exponent(other._glossy_exponent)
{
    _initialize();
}

void BlinnPhongMaterial::_initialize()
{
	assert(_refractive_index.getMin() >= 1);

	_has_transparency = _transmittance.nonZero();

	if (_has_transparency)
	{
		const Color& coefficient_of_extinction_squared = _coefficient_of_extinction*_coefficient_of_extinction;
	
		const Color& refractive_index_minus_one = _refractive_index - 1;
		const Color& refractive_index_plus_one = _refractive_index + 1;

		_glossy_reflectance = (coefficient_of_extinction_squared + refractive_index_minus_one*refractive_index_minus_one)
							 /(coefficient_of_extinction_squared + refractive_index_plus_one*refractive_index_plus_one);
	}

    imp_float max_total = (_lambertian_reflectance + _glossy_reflectance + _transmittance).getMax();
	if (max_total > 1)
	{
		_lambertian_reflectance /= max_total;
		_glossy_reflectance /= max_total;
		_transmittance /= max_total;
	}

    _normalized_lambertian_reflectance = _lambertian_reflectance/IMP_PI;
    _glossy_reflectance_normalization = (8 + _glossy_exponent)/(8*IMP_PI);
    _normalized_glossy_reflectance = _glossy_reflectance*_glossy_reflectance_normalization;
	_average_refractive_index = _refractive_index.getMean();
	
	_has_lambertian_scattering = _lambertian_reflectance.nonZero();
	_has_glossy_scattering = _glossy_reflectance.nonZero();
	_has_emittance = _emitted_radiance.nonZero();
}

void BlinnPhongMaterial::_convert_glossy_reflectance_to_refractive_index()
{
	if (_transmittance.nonZero())
	{
		imp_float sqrt_glossy_r = sqrt(_glossy_reflectance.r);
		imp_float sqrt_glossy_g = sqrt(_glossy_reflectance.g);
		imp_float sqrt_glossy_b = sqrt(_glossy_reflectance.b);

		assert(sqrt_glossy_r < 1 && sqrt_glossy_g < 1 && sqrt_glossy_b < 1);

		_refractive_index.r = (1 + sqrt_glossy_r)/(1 - sqrt_glossy_r);
		_refractive_index.g = (1 + sqrt_glossy_g)/(1 - sqrt_glossy_g);
		_refractive_index.b = (1 + sqrt_glossy_b)/(1 - sqrt_glossy_b);
	}
	else
	{
		_refractive_index = Color::white();
	}

	_coefficient_of_extinction = Color::black();
}

Geometry3D::Vector BlinnPhongMaterial::getRandomGlossyDirection(const Vector& surface_normal,
																const Vector& outgoing_direction) const
{
	imp_float theta = math_util::random()*IMP_TWO_PI;
	imp_float s = math_util::random();

	imp_float y = pow(s, 1/(_glossy_exponent + 1));
	imp_float r = sqrt(1 - y*y);

	return Geometry3D::LinearTransformation::getVectorRotatedFromYAxisToDirection(Vector(r*cos(theta), y, r*sin(theta)),
																				  outgoing_direction.getReflectedAbout(surface_normal));
}

const Reflectance& BlinnPhongMaterial::getLambertianReflectance() const
{
    return _lambertian_reflectance;
}

const Reflectance& BlinnPhongMaterial::getGlossyReflectance() const
{
    return _glossy_reflectance;
}

const Color& BlinnPhongMaterial::getRefractiveIndex() const
{
    return _refractive_index;
}

const Color& BlinnPhongMaterial::getCoefficientOfExtinction() const
{
    return _coefficient_of_extinction;
}

const Color& BlinnPhongMaterial::getTransmittance() const
{
    return _transmittance;
}

const Color& BlinnPhongMaterial::getAttenuation() const
{
    return _attenuation;
}

imp_float BlinnPhongMaterial::getGlossyExponent() const
{
    return _glossy_exponent;
}

void BlinnPhongMaterial::setEmittedRadiance(const Radiance& emitted_radiance)
{
	_emitted_radiance = emitted_radiance;
    _initialize();
}

const Color& BlinnPhongMaterial::getBaseColor() const
{
	return _lambertian_reflectance;
}

const Radiance& BlinnPhongMaterial::getEmittedRadiance() const
{
	return _emitted_radiance;
}

void BlinnPhongMaterial::setLambertianReflectance(const Reflectance& lambertian_reflectance)
{
    _lambertian_reflectance = lambertian_reflectance;
    _initialize();
}

void BlinnPhongMaterial::setGlossyReflectance(const Reflectance& glossy_reflectance)
{
    _glossy_reflectance = glossy_reflectance;
    _initialize();
}

void BlinnPhongMaterial::setRefractiveIndex(const Color& refractive_index)
{
    _refractive_index = refractive_index;
    _initialize();
}

void BlinnPhongMaterial::setCoefficientOfExtinction(const Color& coefficient_of_extinction)
{
    _coefficient_of_extinction = coefficient_of_extinction;
    _initialize();
}

void BlinnPhongMaterial::setTransmittance(const Color& transmittance)
{
    _transmittance = transmittance;
    _initialize();
}

void BlinnPhongMaterial::setAttenuation(const Color& attenuation)
{
    _attenuation = attenuation;
    _initialize();
}

void BlinnPhongMaterial::setGlossyExponent(imp_float glossy_exponent)
{
    _glossy_exponent = glossy_exponent;
    _initialize();
}

Color BlinnPhongMaterial::evaluateFiniteBSDF(const SurfaceElement& surface_element,
                                             const Vector& incoming_direction,
                                             const Vector& outgoing_direction,
											 imp_float cos_incoming_angle) const
{
    if (cos_incoming_angle <= 0 || outgoing_direction.dot(surface_element.shading.normal) <= 0)
    {
        // Incoming ray hits the back of the surface or reflected ray points into the surface
        return Color::black();
    }

	const Color& fresnel_reflectance = getFresnelReflectance(_glossy_reflectance, cos_incoming_angle);
		
	// Lambertian reflectance is scaled down with the remainder of the Fresnel reflectance to conserve energy
	const Color& lambertian_reflectance = (1 - fresnel_reflectance)*surface_element.shading.color*_normalized_lambertian_reflectance;

	if (_glossy_exponent < IMP_FLOAT_INF)
	{
		// Estimate true mirror axis with the half-vector
		imp_float specular_alignment = (incoming_direction + outgoing_direction).normalize().dot(surface_element.shading.normal);

		return lambertian_reflectance + fresnel_reflectance*_glossy_reflectance_normalization*pow(specular_alignment, _glossy_exponent);
	}
	else
	{
		return lambertian_reflectance;
	}
}

bool BlinnPhongMaterial::getReflectiveBSDFImpulse(const SurfaceElement& surface_element,
												  const Vector& outgoing_direction,
												  Impulse& impulse) const
{
	if (_glossy_exponent == IMP_FLOAT_INF)
	{
		imp_float cos_outgoing_angle = std::max<imp_float>(0.001f, outgoing_direction.dot(surface_element.shading.normal));

		impulse.direction = outgoing_direction.getReflectedAbout(surface_element.shading.normal);
		impulse.magnitude = getFresnelReflectance(_glossy_reflectance, cos_outgoing_angle);

		return true;
	}

	return false;
}

bool BlinnPhongMaterial::getRefractiveBSDFImpulse(const SurfaceElement& surface_element,
												  Medium& ray_medium,
												  const Vector& outgoing_direction,
												  Impulse& impulse) const
{
	if (_has_transparency)
	{
		imp_float cos_outgoing_angle;
		imp_float cos_incoming_angle;

		if (ray_medium.material && ray_medium.model_id == surface_element.model->id) // Are we tracing back the ligth ray from inside the model?
		{
			// The minus sign is required because the surface normal
			// points out of the model, but we are on the inside
			const Vector& inside_normal = -surface_element.shading.normal;

			cos_outgoing_angle = std::max<imp_float>(0.001f, outgoing_direction.dot(inside_normal));

			// There is no probability test here because the only thing that could have 
			// happened to the light in this situation is transmission from the outside
			// or total internal reflection from the inside.
			
			// We are assuming that there is air on the outside, with refractive index 1.

			if (_average_refractive_index > 1 &&
				cos_outgoing_angle < sqrt(1 - 1/(_average_refractive_index*_average_refractive_index)))
			{
				// Total internal reflection
				impulse.direction = outgoing_direction.getReflectedAbout(inside_normal);
				impulse.magnitude = Color::white();
			}
			else
			{
				// Light entered this medium from air
				impulse.direction = (-outgoing_direction).getSnellRefracted(inside_normal, cos_outgoing_angle, _average_refractive_index, 1);

				// Find incoming angle with the outside normal
				cos_incoming_angle = std::max<imp_float>(0.001f, impulse.direction.dot(surface_element.shading.normal));
				
				// Compute transmittance for the incoming light
				const Color& fresnel_transmittance = 1 - getFresnelReflectance(_glossy_reflectance, cos_incoming_angle);
				const Color& transmittance = fresnel_transmittance*_transmittance;

				// Radiance is increased with the square of refractive index when entering from air
				impulse.magnitude = transmittance*_refractive_index*_refractive_index;// /transmittance.getMean();

				// The incoming ray must be traced back outside the model
				ray_medium.material = nullptr;
			}
			
			return true;
		}
		else if (!ray_medium.material)
		{
			// We are on the outside of the model, so there is no minus sign 
			// in front of the dot product
			cos_outgoing_angle = std::max<imp_float>(0.001f, outgoing_direction.dot(surface_element.shading.normal));
			
			// Compute transmittance for the incoming light
			const Color& transmittance = _transmittance;

			// Find the direction the light is coming from when it is transmitted from the inside
			cos_outgoing_angle = std::max<imp_float>(0.001f, outgoing_direction.dot(surface_element.shading.normal));
			impulse.direction = (-outgoing_direction).getSnellRefracted(surface_element.shading.normal, cos_outgoing_angle, 1, _average_refractive_index);
				
			// Radiance is decreased with the square of refractive index when leaving to air
			impulse.magnitude = transmittance/(_refractive_index*_refractive_index);
				
			// The incoming ray must be traced back inside the model
			ray_medium.material = this;
			ray_medium.model_id = surface_element.model->id;

			return true;
		}
	}

	return false;
}

bool BlinnPhongMaterial::scatter_back(const SurfaceElement& surface_element,
									  Medium& ray_medium,
									  const Vector& outgoing_direction,
									  Vector& incoming_direction,
									  Color& weight) const
{
	/*
	This method computes an incoming direction for scattered light given the
	outgoing direction and the surface element on which scattering takes place.
	The integration weight that the incoming radiance should be multiplied with
	is also provided.
	*/
	
	imp_float cos_outgoing_angle;
	imp_float cos_incoming_angle;

	imp_float r =  math_util::random();

	if (_has_transparency) // Could the light have been transmitted through the surface?
	{
		if (ray_medium.material && ray_medium.model_id == surface_element.model->id) // Are we tracing back the ligth ray from inside the model?
		{
			// The minus sign is required because the surface normal
			// points out of the model, but we are on the inside
			const Vector& inside_normal = -surface_element.shading.normal;

			cos_outgoing_angle = std::max<imp_float>(0.001f, outgoing_direction.dot(inside_normal));

			// There is no probability test here because the only thing that could have 
			// happened to the light in this situation is transmission from the outside
			// or total internal reflection from the inside.
			
			// We are assuming that there is air on the outside, with refractive index 1.

			if (_average_refractive_index > 1 &&
				cos_outgoing_angle < sqrt(1 - 1/(_average_refractive_index*_average_refractive_index)))
			{
				// Total internal reflection
				incoming_direction = outgoing_direction.getReflectedAbout(inside_normal);
				weight = Color::white();
			}
			else
			{
				// Light entered this medium from air
				incoming_direction = (-outgoing_direction).getSnellRefracted(inside_normal, cos_outgoing_angle, _average_refractive_index, 1);

				// Find incoming angle with the outside normal
				cos_incoming_angle = std::max<imp_float>(0.001f, incoming_direction.dot(surface_element.shading.normal));
				
				// Compute transmittance for the incoming light
				const Color& fresnel_transmittance = 1 - getFresnelReflectance(_glossy_reflectance, cos_incoming_angle);
				const Color& transmittance = fresnel_transmittance*_transmittance;

				// Radiance is increased with the square of refractive index when entering from air
				weight = transmittance*_refractive_index*_refractive_index;// /transmittance.getMean();

				// The incoming ray must be traced back outside the model
				ray_medium.material = nullptr;
			}
			
			return true;
		}
		else if (!ray_medium.material)
		{
			// We are on the outside of the model, so there is no minus sign 
			// in front of the dot product
			cos_outgoing_angle = std::max<imp_float>(0.001f, outgoing_direction.dot(surface_element.shading.normal));
			
			// Find the direction the light would be coming from if it was transmitted from the inside
			//incoming_direction = (-outgoing_direction).getSnellRefracted(surface_element.shading.normal, cos_outgoing_angle, 1, _average_refractive_index);

			// Find corresponding incoming angle with the inside normal
			//cos_incoming_angle = std::max<imp_float>(0.001f, -(incoming_direction.dot(surface_element.shading.normal)));
			
			// Compute transmittance for the incoming light
			//const Color& fresnel_transmittance = 1 - getFresnelReflectance(_glossy_reflectance, cos_incoming_angle);
			//const Color& transmittance = fresnel_transmittance*_transmittance;
			const Color& transmittance = _transmittance;
			
			// Estimate probability of transmission from the inside
			imp_float average_transmittance = transmittance.getMean();
			
			r -= average_transmittance;
			
			// Assume transmission from the inside with this probability
			if (r < 0)
			{
				// Find the direction the light is coming from when it is transmitted from the inside
				cos_outgoing_angle = std::max<imp_float>(0.001f, outgoing_direction.dot(surface_element.shading.normal));
				incoming_direction = (-outgoing_direction).getSnellRefracted(surface_element.shading.normal, cos_outgoing_angle, 1, _average_refractive_index);
				
				// Radiance is decreased with the square of refractive index when leaving to air
				weight = transmittance/(_refractive_index*_refractive_index*average_transmittance);
				
				// The incoming ray must be traced back inside the model
				ray_medium.material = this;
				ray_medium.model_id = surface_element.model->id;

				return true;
			}
		}
		else
		{
			// Transmission between two intersecting models. This case is not handled.
			return false;
		}
	}

	if (_has_lambertian_scattering) // Could the light have been scattered in a Lambertian manner?
	{
		const Color& reflectance = surface_element.shading.color*_lambertian_reflectance;

		// Estimate probability of Lambertian scattering
		imp_float average_lambertian_reflectance = reflectance.getMean();

		r -= average_lambertian_reflectance;

		// Assume Lambertian scattering with this probability
		if (r < 0)
		{
			// Draw incoming direction from the distribution p(w_i) = (w_i . n)/pi
			incoming_direction = Vector::randomCosineWeightedDirectionOnHemisphere(surface_element.shading.normal);
			
			// Compute fresnel reflectance for the incoming light
			cos_incoming_angle = std::max<imp_float>(0.001f, incoming_direction.dot(surface_element.shading.normal));
			const Color& fresnel_reflectance = getFresnelReflectance(_glossy_reflectance, cos_incoming_angle);

			weight = (1 - fresnel_reflectance)*reflectance/average_lambertian_reflectance;

			//weight = reflectance/average_lambertian_reflectance;

			return true;
		}
	}

	if (_has_glossy_scattering && _glossy_exponent < IMP_FLOAT_INF) // Could the light have been scattered in a glossy manner?
	{
		// Draw incoming direction from the distribution p(w_i, w_o) = (s+1)/(2*pi)*(w_o . w_o.reflected(n))^s
		incoming_direction = getRandomGlossyDirection(surface_element.shading.normal, outgoing_direction);

		const Color& reflectance = _glossy_reflectance;
			
		// Compute fresnel reflectance for the incoming light
		cos_incoming_angle = std::max<imp_float>(0.001f, incoming_direction.dot(surface_element.shading.normal));
		const Color& fresnel_reflectance = getFresnelReflectance(reflectance, cos_incoming_angle);
			
		// Estimate probability of glossy scattering
		imp_float average_fresnel_reflectance = fresnel_reflectance.getMean();
			
		r -= average_fresnel_reflectance;
			
		// Perform glossy scattering with this probability
		if (r < 0)
		{
			// The remainder of the proportionality constant f_glossy(w_i, w_0)/p(w_i, w_o) should in
			// principle be included, but it is very close to 1 when the glossy exponent is significant
			weight = fresnel_reflectance*cos_incoming_angle/average_fresnel_reflectance;
			return true;
		}
	}
	else if (_glossy_exponent == IMP_FLOAT_INF) // Mirror scattering
	{
		// Find the direction the light would be coming from if it was mirror reflected
		incoming_direction = outgoing_direction.getReflectedAbout(surface_element.shading.normal);
			
		// Compute fresnel reflectance for the incoming light
		cos_incoming_angle = std::max<imp_float>(0.001f, incoming_direction.dot(surface_element.shading.normal));
		const Color& fresnel_reflectance = getFresnelReflectance(_glossy_reflectance, cos_incoming_angle);
			
		// Estimate probability of mirror scattering
		imp_float average_fresnel_reflectance = fresnel_reflectance.getMean();
			
		r -= average_fresnel_reflectance;
			
		// Perform mirror scattering with this probability
		if (r < 0)
		{
			weight = fresnel_reflectance/average_fresnel_reflectance;
			return true;
		}
	}

	return false;
}

Color BlinnPhongMaterial::getAttenuationFactor(imp_float distance) const
{
	return Color((_attenuation.r != 0)? exp(-_attenuation.r*distance) : 1,
				 (_attenuation.g != 0)? exp(-_attenuation.g*distance) : 1,
				 (_attenuation.b != 0)? exp(-_attenuation.b*distance) : 1);
}

bool BlinnPhongMaterial::isTransparent() const
{
	return _has_transparency;
}

bool BlinnPhongMaterial::isEmitter() const
{
	return _has_emittance;
}

} // Rendering3D
} // Impact
