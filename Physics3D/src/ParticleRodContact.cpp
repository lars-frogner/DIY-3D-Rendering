#include "ParticleRodContact.hpp"
#include <cassert>

namespace Impact {
namespace Physics3D {

ParticleRodContact::ParticleRodContact(Particle* new_particle_1,
									   Particle* new_particle_2,
									   imp_float new_length)
	: ParticleLinkedContact(new_particle_1, new_particle_2),
	  _length(new_length) {}

void ParticleRodContact::generateContacts(std::list<ParticleContact>& contact_list) const
{
	Vector separation = getCurrentSeparation();
	imp_float distance = separation.getLength();

	if (distance == _length)
		return;

	separation /= distance;

	if (distance > _length)
	{
		contact_list.emplace_back(_particle_1,
								   _particle_2,
								   -separation,
								   distance - _length,
								   0.0f);
	}
	else
	{
		contact_list.emplace_back(_particle_1,
								   _particle_2,
								   separation,
								   _length - distance,
								   0.0f);
	}
}

} // Physics3D
} // Impact
