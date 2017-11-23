#include "ParticleGravityForce.hpp"

namespace Impact {
namespace Physics3D {

ParticleGravityForce::ParticleGravityForce(const Vector& new_gravitational_acceleration)
	: _gravitational_acceleration(new_gravitational_acceleration) {}

void ParticleGravityForce::addForce(Particle* particle, imp_float duration)
{
	if (particle->hasInfiniteMass())
		return;

	particle->addForce(_gravitational_acceleration*(particle->getMass()));
}

} // Physics3D
} // Impact
