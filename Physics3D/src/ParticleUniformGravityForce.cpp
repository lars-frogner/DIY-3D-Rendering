#include "ParticleUniformGravityForce.hpp"

namespace Impact {
namespace Physics3D {

ParticleUniformGravityForce::ParticleUniformGravityForce(const Vector& new_gravitational_acceleration)
	: _gravitational_acceleration(new_gravitational_acceleration) {}

void ParticleUniformGravityForce::addForce(Particle* particle, imp_float duration)
{
	if (particle->hasInfiniteMass())
		return;

	particle->addForce(_gravitational_acceleration*(particle->getMass()));
}

} // Physics3D
} // Impact
