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
      _attenuation(Color::black()),
      _glossy_exponent(0),
	  _has_transparency(false)
{
    _initialize();
}

BlinnPhongMaterial::BlinnPhongMaterial(const Reflectance& new_lambertian_reflectance,
									   const Color& new_glossy_reflectance,
									   imp_float new_glossy_exponent)
    : _lambertian_reflectance(new_lambertian_reflectance),
      _glossy_reflectance(new_glossy_reflectance),
      _glossy_exponent(new_glossy_exponent),
      _attenuation(Color::black()),
	  _has_transparency(false)
{
    _convert_glossy_reflectance_to_refractive_index();
    _initialize();
}

BlinnPhongMaterial::BlinnPhongMaterial(const Reflectance& new_lambertian_reflectance,
									   const Color& new_refractive_index,
									   const Color& new_coefficient_of_extinction,
									   const Color& new_attenuation,
									   imp_float new_glossy_exponent)
    : _lambertian_reflectance(new_lambertian_reflectance),
      _refractive_index(new_refractive_index),
      _coefficient_of_extinction(new_coefficient_of_extinction),
      _attenuation(new_attenuation),
	  _glossy_exponent(new_glossy_exponent),
	  _has_transparency(true)
{
    _initialize();
}

BlinnPhongMaterial::BlinnPhongMaterial(const BlinnPhongMaterial& other)
    : _lambertian_reflectance(other._lambertian_reflectance),
      _refractive_index(other._refractive_index),
      _coefficient_of_extinction(other._coefficient_of_extinction),
      _attenuation(other._attenuation),
      _glossy_exponent(other._glossy_exponent),
	  _has_transparency(other._has_transparency)
{
    _initialize();
}

void BlinnPhongMaterial::_initialize()
{
	const Color& coefficient_of_extinction_squared = _coefficient_of_extinction*_coefficient_of_extinction;
	
	const Color& refractive_index_minus_one = _refractive_index - 1;
	const Color& refractive_index_plus_one = _refractive_index + 1;

	_glossy_reflectance = (coefficient_of_extinction_squared + refractive_index_minus_one*refractive_index_minus_one)
						 /(coefficient_of_extinction_squared + refractive_index_plus_one*refractive_index_plus_one);

    assert((_lambertian_reflectance + _glossy_reflectance).getMax() <= 1);

    _normalized_lambertian_reflectance = _lambertian_reflectance/IMP_PI;
    _glossy_reflectance_normalization = (8 + _glossy_exponent)/(8*IMP_PI);
    _normalized_glossy_reflectance = _glossy_reflectance*_glossy_reflectance_normalization;
}

void BlinnPhongMaterial::_convert_glossy_reflectance_to_refractive_index()
{
	imp_float max_total = (_lambertian_reflectance + _glossy_reflectance).getMax()*1.00001f;

	if (max_total > 1)
	{
		_lambertian_reflectance /= max_total;
		_glossy_reflectance /= max_total;
	}

	imp_float sqrt_glossy_r = sqrt(_glossy_reflectance.r);
	imp_float sqrt_glossy_g = sqrt(_glossy_reflectance.g);
	imp_float sqrt_glossy_b = sqrt(_glossy_reflectance.b);

	assert(sqrt_glossy_r < 1 && sqrt_glossy_g < 1 && sqrt_glossy_b < 1);

    _refractive_index.r = (1 + sqrt_glossy_r)/(1 - sqrt_glossy_r);
    _refractive_index.g = (1 + sqrt_glossy_g)/(1 - sqrt_glossy_g);
    _refractive_index.b = (1 + sqrt_glossy_b)/(1 - sqrt_glossy_b);

	_coefficient_of_extinction = Color::black();
}

Geometry3D::Vector BlinnPhongMaterial::getRandomGlossyDirection(const Vector& surface_normal,
																const Vector& incoming_direction) const
{
	imp_float theta = math_util::random()*IMP_TWO_PI;
	imp_float s = math_util::random();

	imp_float y = pow(s, 1/(_glossy_exponent + 1));
	imp_float r = sqrt(1 - y*y);

	return Geometry3D::LinearTransformation::getVectorRotatedFromYAxisToDirection(Vector(r*cos(theta), y, r*sin(theta)),
																				  incoming_direction.getReflectedAbout(surface_normal));
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

Color BlinnPhongMaterial::evaluateFiniteBSDF(const Vector& surface_normal,
                                             const Vector& incoming_direction,
                                             const Vector& outgoing_direction,
											 imp_float cos_incoming_angle) const
{
    if (cos_incoming_angle <= 0 || outgoing_direction.dot(surface_normal) <= 0)
    {
        // Incoming ray hits the back of the surface or reflected ray points into the surface
        return Color::black();
    }

	if (use_fresnel)
	{
		const Color& fresnel_reflectance = getFresnelReflectance(_glossy_reflectance, cos_incoming_angle);

		if (_glossy_exponent < IMP_FLOAT_INF)
		{
			imp_float specular_alignment = (incoming_direction + outgoing_direction).normalize().dot(surface_normal);

			return (1 - fresnel_reflectance)*_normalized_lambertian_reflectance + fresnel_reflectance*_glossy_reflectance_normalization*pow(specular_alignment, _glossy_exponent);
		}
		else
		{
			return (1 - fresnel_reflectance)*_normalized_lambertian_reflectance;
		}
	}
	else
	{
		if (_glossy_exponent < IMP_FLOAT_INF)
		{
			imp_float specular_alignment = (incoming_direction + outgoing_direction).normalize().dot(surface_normal);

			return _normalized_lambertian_reflectance + _normalized_glossy_reflectance*pow(specular_alignment, _glossy_exponent);
		}
		else
		{
			return _normalized_lambertian_reflectance;
		}
	}
}

void BlinnPhongMaterial::getBSDFImpulses(const Vector& surface_normal,
                                         const Vector& incoming_direction,
										 std::vector<Impulse>& impulses) const
{
	Impulse impulse;

	imp_float cos_incoming_angle = std::max<imp_float>(0.001f, incoming_direction.dot(surface_normal));

	const Color& fresnel_reflectance = getFresnelReflectance(_glossy_reflectance, cos_incoming_angle);

	if (_glossy_exponent == IMP_FLOAT_INF)
	{
		impulse.direction = incoming_direction.getReflectedAbout(surface_normal);
		impulse.magnitude = fresnel_reflectance;
		
		impulses.push_back(impulse);
	}

	if (_has_transparency)
	{
		impulse.direction = (-incoming_direction).getSnellRefracted(surface_normal, cos_incoming_angle, 1, _refractive_index.getMean());
		impulse.magnitude = (1 - fresnel_reflectance)*_refractive_index*_refractive_index;
		
		impulses.push_back(impulse);
	}
}

bool BlinnPhongMaterial::scatter(const SurfaceElement& surface_element,
								 Medium& ray_medium,
								 const Vector& incoming_direction,
								 Vector& outgoing_direction,
								 Color& weight) const
{
	Color fresnel_reflectance;
	imp_float cos_incoming_angle;
	bool has_computed_fresnel_reflectance = false;
	imp_float r =  math_util::random();

	if (_lambertian_reflectance.nonZero())
	{
		imp_float average_lambertian_reflectance = _lambertian_reflectance.getMean();

		r -= average_lambertian_reflectance;

		if (r < 0)
		{
			outgoing_direction = Vector::randomCosineWeightedDirectionOnHemisphere(surface_element.shading.normal);
			weight = _lambertian_reflectance/average_lambertian_reflectance;

			return true;
		}
	}

	if (_glossy_reflectance.nonZero())
	{
		cos_incoming_angle = std::max<imp_float>(0.001f, incoming_direction.dot(surface_element.shading.normal));

		fresnel_reflectance = getFresnelReflectance(_glossy_reflectance, cos_incoming_angle);
		has_computed_fresnel_reflectance = true;

		imp_float average_fresnel_reflectance = fresnel_reflectance.getMean();

		r -= average_fresnel_reflectance;

		if (r < 0)
		{
			if (_glossy_exponent < IMP_FLOAT_INF)
			{
				outgoing_direction = getRandomGlossyDirection(surface_element.shading.normal, incoming_direction);
				weight = fresnel_reflectance/average_fresnel_reflectance;
			}
			else
			{
				outgoing_direction = incoming_direction.getReflectedAbout(surface_element.shading.normal);
				weight = fresnel_reflectance/average_fresnel_reflectance;
			}

			return true;
		}
	}

	if (_has_transparency)
	{
		Color fresnel_transmittance;

		if (has_computed_fresnel_reflectance)
		{
			fresnel_transmittance = 1 - fresnel_reflectance;
		}
		else
		{	
			cos_incoming_angle = std::max<imp_float>(0.001f, incoming_direction.dot(surface_element.shading.normal));
			imp_float one_minus_cos_incoming_angle = 1 - cos_incoming_angle;
			fresnel_transmittance.r = 1 - one_minus_cos_incoming_angle*one_minus_cos_incoming_angle*one_minus_cos_incoming_angle*one_minus_cos_incoming_angle*one_minus_cos_incoming_angle;
			fresnel_transmittance.g = fresnel_transmittance.r;
			fresnel_transmittance.b = fresnel_transmittance.r;
		}

		imp_float average_fresnel_transmittance = fresnel_transmittance.getMean();

		if (false && ray_medium.material) // Are we on the inside of a medium?
		{
			if (ray_medium.model_id == surface_element.model_id)
			{
				// Ray leaves this medium to air
				outgoing_direction = (-incoming_direction).getSnellRefracted(surface_element.shading.normal, cos_incoming_angle, _refractive_index.getMean(), 1);
					
				// Total internal reflection
				if (!outgoing_direction.nonZero())
					return false;

				weight = fresnel_transmittance/(_refractive_index*_refractive_index*average_fresnel_transmittance);
				ray_medium.material = nullptr;
			}
			else
			{
				// Ray enters this medium from another medium
				const Color& other_refractive_index = ray_medium.material->getRefractiveIndex();
				outgoing_direction = (-incoming_direction).getSnellRefracted(surface_element.shading.normal, cos_incoming_angle, other_refractive_index.getMean(), _refractive_index.getMean());

				// Total internal reflection
				if (!outgoing_direction.nonZero())
					return false;

				weight = _refractive_index*_refractive_index*fresnel_transmittance/(other_refractive_index*other_refractive_index*average_fresnel_transmittance);
				ray_medium.material = this;
				ray_medium.model_id = surface_element.model_id;
			}
		}
		else
		{
			// Ray enters this medium from air
			outgoing_direction = (-incoming_direction).getSnellRefracted(surface_element.shading.normal, cos_incoming_angle, 1, _refractive_index.getMean());
			weight = _refractive_index*_refractive_index*fresnel_transmittance/average_fresnel_transmittance;
			ray_medium.material = this;
			ray_medium.model_id = surface_element.model_id;
		}

		return true;
	}

	return false;
}

void BlinnPhongMaterial::attenuate(imp_float distance, Radiance& radiance) const
{
	
}

} // Rendering3D
} // Impact
