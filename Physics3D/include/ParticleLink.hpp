#pragma once
#include "precision.hpp"
#include "Particle.hpp"
#include "ParticleContact.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleLink {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	Particle* _particle_1;
	Particle* _particle_2;
	
	Vector getCurrentSeparation() const;
	imp_float getCurrentDistance() const;

public:
	virtual imp_uint fillContact(ParticleContact* first_contact,
								 imp_uint max_available_contacts) const = 0;
};

} // Physics3D
} // Impact
