#include "ParticleParticleContact.hpp"
#include <cassert>
#include <cmath>

namespace Impact {
namespace Physics3D {

ParticleParticleContact::ParticleParticleContact(Particle* new_particle_1,
												 Particle* new_particle_2,
												 imp_float new_restitution_coef)
	: ParticleLinkedContact(new_particle_1, new_particle_2),
	  _restitution_coef(new_restitution_coef),
	  _minimum_distance(new_particle_1->getRadius() + new_particle_2->getRadius()) {}

imp_uint ParticleParticleContact::generateContacts(imp_uint n_available_contacts,
												   ParticleContact* first_available_contact) const
{
	assert(n_available_contacts >= 1);

	Vector separation = getCurrentSeparation();
	imp_float distance = separation.getLength();

	if (distance > _minimum_distance)
		return 0;

	separation /= distance;

	first_available_contact->setParticles(_particle_1, _particle_2);
	first_available_contact->setContactNormal(separation);
	first_available_contact->setPenetrationDepth(_minimum_distance - distance);
	first_available_contact->setRestitutionCoefficient(_restitution_coef);

	return 1;
}

} // Physics3D
} // Impact
