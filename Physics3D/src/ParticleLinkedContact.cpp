#include "ParticleLinkedContact.hpp"

namespace Impact {
namespace Physics3D {

ParticleLinkedContact::ParticleLinkedContact(Particle* new_particle_1,
										     Particle* new_particle_2)
	: _particle_1(new_particle_1),
	  _particle_2(new_particle_2) {}

Geometry3D::Vector ParticleLinkedContact::getCurrentSeparation() const
{
	return _particle_1->getPosition() - _particle_2->getPosition();
}

imp_float ParticleLinkedContact::getCurrentDistance() const
{
	Vector separation = _particle_1->getPosition() - _particle_2->getPosition();
	return separation.getLength();
}

std::vector<Particle*> ParticleLinkedContact::getInvolvedParticles() const
{
	std::vector<Particle*> particles(2);
	particles[0] = _particle_1;
	particles[1] = _particle_2;
	return particles;
}

} // Physics3D
} // Impact
