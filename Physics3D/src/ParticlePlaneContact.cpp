#include "ParticlePlaneContact.hpp"
#include <cassert>
#include <cmath>

namespace Impact {
namespace Physics3D {

ParticlePlaneContact::ParticlePlaneContact(Particle* new_particle,
										   const Plane& new_plane,
										   imp_float new_restitution_coef)
	: _particle(new_particle),
	  _plane(new_plane),
	  _restitution_coef(new_restitution_coef) {}

std::vector<Particle*> ParticlePlaneContact::getInvolvedParticles() const
{
	std::vector<Particle*> particles(1);
	particles[0] = _particle;
	return particles;
}

imp_uint ParticlePlaneContact::generateContacts(imp_uint n_available_contacts,
												ParticleContact* first_available_contact) const
{
	assert(n_available_contacts >= 1);

	imp_float separation = _plane.getNormalVector().dot(_particle->getPosition() - _plane.origin);
	imp_float penetration_depth = _particle->getRadius() - abs(separation);

	if (penetration_depth < 0)
		return 0;

	first_available_contact->setParticles(_particle, nullptr);

	if (separation >= 0)
		first_available_contact->setContactNormal(_plane.getNormalVector());
	else
		first_available_contact->setContactNormal(-_plane.getNormalVector());

	first_available_contact->setPenetrationDepth(penetration_depth);
	first_available_contact->setRestitutionCoefficient(_restitution_coef);

	return 1;
}

} // Physics3D
} // Impact
