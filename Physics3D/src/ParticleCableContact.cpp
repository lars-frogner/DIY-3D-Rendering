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

void ParticleCableContact::generateContacts(std::list<ParticleContact>& contact_list) const
{
	Vector separation = getCurrentSeparation();
	imp_float distance = separation.getLength();

	if (distance < _max_length)
		return;

	separation /= distance;

	contact_list.emplace_back(_particle_1,
							   _particle_2,
							   -separation,
							   distance - _max_length,
						       _restitution_coef);
}

} // Physics3D
} // Impact
