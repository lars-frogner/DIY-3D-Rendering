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

void ParticleParticleContact::generateContacts(std::list<ParticleContact>& contact_list) const
{
	Vector separation = getCurrentSeparation();
	imp_float distance = separation.getLength();

	if (distance > _minimum_distance)
		return;

	separation /= distance;

	contact_list.emplace_back(_particle_1,
							   _particle_2,
							   separation,
							   _minimum_distance - distance,
						       _restitution_coef);
}

} // Physics3D
} // Impact
