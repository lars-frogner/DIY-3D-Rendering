#pragma once
#include "precision.hpp"
#include "Particle.hpp"
#include "ParticleForceGenerator.hpp"
#include <vector>

namespace Impact {
namespace Physics3D {

class ParticleWorld;

class ParticleForceRegistry {

friend ParticleWorld;

protected:

	struct ParticleForceRegistration
	{
		Particle* particle;
		ParticleForceGenerator* force_generator;
	};
	
	friend inline bool operator==(const ParticleForceRegistration& lhs, const ParticleForceRegistration& rhs);

	typedef std::vector<ParticleForceRegistration> Registry;

	Registry _registrations;

	void applyForces(imp_float duration);

	void addForceGenerator(Particle* particle, ParticleForceGenerator* force_generator);
	void removeForceGenerator(Particle* particle, ParticleForceGenerator* force_generator);
	void clearForceGenerators();
};

} // Physics3D
} // Impact
