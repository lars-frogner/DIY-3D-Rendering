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

void ParticlePlaneContact::generateContacts(std::list<ParticleContact>& contact_list) const
{
	imp_float separation = _plane.getNormalVector().dot(_particle->getPosition() - _plane.origin);
	imp_float penetration_depth = _particle->getRadius() - abs(separation);

	if (penetration_depth < 0)
		return;

	if (separation >= 0)
	{
		contact_list.emplace_back(_particle,
								   nullptr,
								   _plane.getNormalVector(),
								   penetration_depth,
								   _restitution_coef);
	}
	else
	{
		contact_list.emplace_back(_particle,
								   nullptr,
								   -_plane.getNormalVector(),
								   penetration_depth,
								   _restitution_coef);
	}
}

} // Physics3D
} // Impact
