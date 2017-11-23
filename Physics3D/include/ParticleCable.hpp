#pragma once
#include "precision.hpp"
#include "ParticleLink.hpp"
#include "Particle.hpp"
#include "ParticleContact.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleCable : public ParticleLink {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	imp_float _max_length;
	imp_float _restitution_coef;

public:
	ParticleCable(imp_float new_max_length,
				  imp_float new_restitution_coef);

	virtual imp_uint fillContact(ParticleContact* first_contact,
								 imp_uint max_available_contacts) const;
};

} // Physics3D
} // Impact
