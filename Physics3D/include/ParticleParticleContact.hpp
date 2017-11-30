#pragma once
#include "precision.hpp"
#include "ParticleLinkedContact.hpp"
#include "Particle.hpp"
#include "ParticleContact.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include <vector>

namespace Impact {
namespace Physics3D {

class ParticleParticleContact : public ParticleLinkedContact {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	imp_float _restitution_coef;
	imp_float _minimum_distance;

public:
	ParticleParticleContact(Particle* new_particle_1,
							Particle* new_particle_2,
							imp_float new_restitution_coef);

	virtual imp_uint generateContacts(imp_uint n_available_contacts,
									  ParticleContact* first_available_contact) const;
};

} // Physics3D
} // Impact
