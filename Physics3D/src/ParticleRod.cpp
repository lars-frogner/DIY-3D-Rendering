#include "ParticleRod.hpp"
#include <cassert>

namespace Impact {
namespace Physics3D {

ParticleRod::ParticleRod(imp_float new_length)
	: _length(new_length) {}

imp_uint ParticleRod::fillContact(ParticleContact* first_contact,
								  imp_uint max_available_contacts) const
{
	assert(max_available_contacts >= 2);

	Vector separation = getCurrentSeparation();
	imp_float distance = separation.getLength();

	if (distance == _length)
		return 0;

	separation /= distance;
	
	first_contact->setParticles(_particle_1, _particle_2);
	first_contact->setContactNormal(separation);
	first_contact->setPenetrationDepth(distance - _length);
	first_contact->setRestitutionCoefficient(0);

	ParticleContact* second_contact = first_contact + 1;

	assert(second_contact);
	
	second_contact->setParticles(_particle_1, _particle_2);
	second_contact->setContactNormal(-separation);
	second_contact->setPenetrationDepth(_length - distance);
	second_contact->setRestitutionCoefficient(0);

	return 2;
}

} // Physics3D
} // Impact
