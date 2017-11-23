#pragma once
#include "precision.hpp"
#include "ParticleLink.hpp"
#include "Particle.hpp"
#include "ParticleContact.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleRod : public ParticleLink {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	imp_float _length;

public:
	ParticleRod(imp_float new_length);

	virtual imp_uint fillContact(ParticleContact* first_contact,
								 imp_uint max_available_contacts) const;
};

} // Physics3D
} // Impact
