#pragma once
#include "precision.hpp"
#include "ParticleLinkedContact.hpp"
#include "Particle.hpp"
#include "ParticleContact.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleCableContact : public ParticleLinkedContact {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	imp_float _max_length;
	imp_float _restitution_coef;

public:
	ParticleCableContact(Particle* new_particle_1,
						 Particle* new_particle_2,
						 imp_float new_max_length,
						 imp_float new_restitution_coef);

	virtual imp_uint generateContacts(imp_uint n_available_contacts,
									  ParticleContact* first_available_contact) const;
};

} // Physics3D
} // Impact
