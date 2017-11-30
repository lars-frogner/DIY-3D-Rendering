#include "ParticleCableContact.hpp"
#include <cassert>

namespace Impact {
namespace Physics3D {

ParticleCableContact::ParticleCableContact(Particle* new_particle_1,
										   Particle* new_particle_2,
										   imp_float new_max_length,
										   imp_float new_restitution_coef)
	: ParticleLinkedContact(new_particle_1, new_particle_2),
	  _max_length(new_max_length),
	  _restitution_coef(new_restitution_coef) {}

imp_uint ParticleCableContact::generateContacts(imp_uint n_available_contacts,
						   						ParticleContact* first_available_contact) const
{
	assert(n_available_contacts >= 1);

	Vector separation = getCurrentSeparation();
	imp_float distance = separation.getLength();

	if (distance < _max_length)
		return 0;

	separation /= distance;

	first_available_contact->setParticles(_particle_1, _particle_2);
	first_available_contact->setContactNormal(-separation);
	first_available_contact->setPenetrationDepth(distance - _max_length);
	first_available_contact->setRestitutionCoefficient(_restitution_coef);

	return 1;
}

} // Physics3D
} // Impact
