#include "ParticleForceRegistry.hpp"
#include <algorithm>

namespace Impact {
namespace Physics3D {

inline bool operator==(const ParticleForceRegistry::ParticleForceRegistration& lhs, const ParticleForceRegistry::ParticleForceRegistration& rhs)
{
	return lhs.particle == rhs.particle && lhs.force_generator == rhs.force_generator;
}

void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator* force_generator)
{
	ParticleForceRegistration registration = {particle, force_generator};
	_registrations.push_back(registration);
}

void ParticleForceRegistry::remove(Particle* particle, ParticleForceGenerator* force_generator)
{
	ParticleForceRegistration registration = {particle, force_generator};
	_registrations.erase(std::remove(_registrations.begin(), _registrations.end(), registration), _registrations.end());
}

void ParticleForceRegistry::clear()
{
	_registrations.clear();
}

void ParticleForceRegistry::addForces(imp_float duration)
{
	for (Registry::iterator iter = _registrations.begin(); iter != _registrations.end(); iter++)
	{
		iter->force_generator->addForce(iter->particle, duration);
	}
}

} // Physics3D
} // Impact
