#pragma once
#include "precision.hpp"
#include "Particle.hpp"

namespace Impact {
namespace Physics3D {

class ParticleForceGenerator {

public:
	virtual void addForce(Particle* particle, imp_float duration) = 0;

};

} // Physics3D
} // Impact
