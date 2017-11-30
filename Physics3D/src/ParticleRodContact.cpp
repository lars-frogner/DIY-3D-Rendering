#include "ParticleRodContact.hpp"
#include <cassert>

namespace Impact {
namespace Physics3D {

ParticleRodContact::ParticleRodContact(Particle* new_particle_1,
									   Particle* new_particle_2,
									   imp_float new_length)
	: ParticleLinkedContact(new_particle_1, new_particle_2),
	  _length(new_length) {}

imp_uint ParticleRodContact::generateContacts(imp_uint n_available_contacts,
											  ParticleContact* first_available_contact) const
{
	assert(n_available_contacts >= 1);

	Vector separation = getCurrentSeparation();
	imp_float distance = separation.getLength();

	if (distance == _length)
		return 0;

	separation /= distance;

	if (distance > _length)
	{
		first_available_contact->setContactNormal(-separation);
		first_available_contact->setPenetrationDepth(distance - _length);
	}
	else
	{
		first_available_contact->setContactNormal(separation);
		first_available_contact->setPenetrationDepth(_length - distance);
	}
	
	first_available_contact->setParticles(_particle_1, _particle_2);
	first_available_contact->setRestitutionCoefficient(0);

	return 1;
}

} // Physics3D
} // Impact
