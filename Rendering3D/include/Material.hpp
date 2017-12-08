#pragma once
#include "precision.hpp"
#include "Color.hpp"
#include "Vector3.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

// Forward declaration of SurfaceElement class
class SurfaceElement;

// Forward declaration of Medium struct
struct Medium;

class Material {
    
private:
    typedef Geometry3D::Vector Vector;

protected:
	Radiance _emitted_radiance = Radiance::black();

public:
	struct Impulse
	{
		Vector direction;
		Color magnitude;
	};

	bool use_fresnel = true;

	static Reflectance getFresnelReflectance(const Color& normal_incidence_reflection, imp_float cos_incoming_angle);

    virtual Color evaluateFiniteBSDF(const Vector& surface_normal,
									 const Vector& incoming_direction,
									 const Vector& outgoing_direction,
									 imp_float cos_incoming_angle) const = 0;

	virtual bool getReflectiveBSDFImpulse(const Vector& surface_normal,
										  const Vector& outgoing_direction,
										  Impulse& impulse) const = 0;

	virtual bool getRefractiveBSDFImpulse(const SurfaceElement& surface_element,
										  Medium& ray_medium,
										  const Vector& outgoing_direction,
										  Impulse& impulse) const = 0;

	virtual bool scatter_back(const SurfaceElement& surface_element,
							  Medium& ray_medium,
							  const Vector& outgoing_direction,
							  Vector& incoming_direction,
							  Color& weight) const = 0;

	virtual bool isTransparent() const = 0;

	virtual const Color& getTransmittance() const = 0;
	
	virtual Color getAttenuationFactor(imp_float distance) const = 0;
	
	virtual const Color& getRefractiveIndex() const = 0;

	virtual const Color& getBaseColor() const = 0;
	virtual void setEmittedRadiance(const Radiance&) = 0;
	virtual const Radiance& getEmittedRadiance() const = 0;
};

struct Medium
{
	const Material* material = nullptr;
	imp_uint model_id;
};

} // Rendering3D
} // Impact
