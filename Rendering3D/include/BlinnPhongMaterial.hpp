#pragma once
#include "precision.hpp"
#include "Material.hpp"
#include "Color.hpp"
#include "SurfaceElement.hpp"
#include "Vector3.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class BlinnPhongMaterial : public Material {
    
private:
    typedef Geometry3D::Vector Vector;

protected:
    Reflectance _lambertian_reflectance;
    Color _refractive_index;
    Color _coefficient_of_extinction;
	Color _transmittance;
	Color _attenuation;
    imp_float _glossy_exponent;

	bool _has_lambertian_scattering;
	bool _has_glossy_scattering;
	bool _has_transparency;
	bool _has_emittance;
	
    Reflectance _glossy_reflectance;
    Color _normalized_lambertian_reflectance;
    imp_float _glossy_reflectance_normalization;
	Color _normalized_glossy_reflectance;
	imp_float _average_refractive_index;
	
    void _convert_glossy_reflectance_to_refractive_index();
    void _initialize();

	Vector getRandomGlossyDirection(const Vector& surface_normal,
									const Vector& outgoing_direction) const;

public:
    BlinnPhongMaterial();

    BlinnPhongMaterial(const Reflectance& new_lambertian_reflectance,
                       const Color& new_glossy_reflectance,
                       imp_float new_glossy_exponent);

    BlinnPhongMaterial(const Reflectance& new_lambertian_reflectance,
                       const Color& new_refractive_index,
                       const Color& new_coefficient_of_extinction,
                       const Color& new_transmittance,
                       const Color& new_attenuation,
                       imp_float new_glossy_exponent);

    BlinnPhongMaterial(const BlinnPhongMaterial& other);

    const Reflectance& getLambertianReflectance() const;
    const Reflectance& getGlossyReflectance() const;
    const Color& getRefractiveIndex() const;
    const Color& getCoefficientOfExtinction() const;
    const Color& getTransmittance() const;
    const Color& getAttenuation() const;
    imp_float getGlossyExponent() const;
	const Radiance& getEmittedRadiance() const;
	
	const Color& getBaseColor() const;

	void setEmittedRadiance(const Radiance& emitted_radiance);

    void setLambertianReflectance(const Reflectance& lambertian_reflectance);
    void setGlossyReflectance(const Reflectance& glossy_reflectance);
    void setRefractiveIndex(const Color& refractive_index);
    void setCoefficientOfExtinction(const Color& coefficient_of_extinction);
    void setTransmittance(const Color& transmittance);
    void setAttenuation(const Color& attenuation);
    void setGlossyExponent(imp_float glossy_exponent);

    Color evaluateFiniteBSDF(const Vector& surface_normal,
					         const Vector& incoming_direction,
							 const Vector& outgoing_direction,
							 imp_float cos_incoming_angle) const;

	bool getReflectiveBSDFImpulse(const Vector& surface_normal,
								  const Vector& outgoing_direction,
								  Impulse& impulse) const;

	bool getRefractiveBSDFImpulse(const SurfaceElement& surface_element,
								  Medium& ray_medium,
								  const Vector& outgoing_direction,
								  Impulse& impulse) const;

	bool scatter_back(const SurfaceElement& surface_element,
					  Medium& ray_medium,
					  const Vector& outgoing_direction,
					  Vector& incoming_direction,
					  Color& weight) const;

	Color getAttenuationFactor(imp_float distance) const;

	bool isTransparent() const;

	bool isEmitter() const;
};

} // Rendering3D
} // Impact
