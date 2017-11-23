#pragma once
#include "precision.hpp"
#include "Particle.hpp"
#include "ParticleForceGenerator.hpp"
#include <vector>

namespace Impact {
namespace Physics3D {

class ParticleForceRegistry {

protected:

	struct ParticleForceRegistration
	{
		Particle* particle;
		ParticleForceGenerator* force_generator;
	};
	
	friend inline bool operator==(const ParticleForceRegistration& lhs, const ParticleForceRegistration& rhs);

	typedef std::vector<ParticleForceRegistration> Registry;

	Registry _registrations;

public:

	void add(Particle* particle, ParticleForceGenerator* force_generator);
	void remove(Particle* particle, ParticleForceGenerator* force_generator);
	void clear();

	void addForces(imp_float duration);
};

} // Physics3D
} // Impact
