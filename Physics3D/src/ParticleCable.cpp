#include "ParticleCable.hpp"
#include <cassert>

namespace Impact {
namespace Physics3D {

ParticleCable::ParticleCable(imp_float new_max_length,
							 imp_float new_restitution_coef)
	: _max_length(new_max_length),
	  _restitution_coef(new_restitution_coef) {}

imp_uint ParticleCable::fillContact(ParticleContact* first_contact,
								    imp_uint max_available_contacts) const
{
	assert(max_available_contacts >= 1);

	Vector separation = getCurrentSeparation();
	imp_float distance = separation.getLength();

	if (distance < _max_length)
		return 0;

	separation /= distance;

	first_contact->setParticles(_particle_1, _particle_2);
	first_contact->setContactNormal(separation);
	first_contact->setPenetrationDepth(distance - _max_length);
	first_contact->setRestitutionCoefficient(_restitution_coef);

	return 1;
}

} // Physics3D
} // Impact
