#pragma once
#include "precision.hpp"
#include "Particle.hpp"
#include "ParticleContact.hpp"
#include <vector>
#include <list>

namespace Impact {
namespace Physics3D {

class ParticleContactGenerator {

public:
	virtual std::vector<Particle*> getInvolvedParticles() const = 0;

	virtual void generateContacts(std::list<ParticleContact>& contact_list) const = 0;
};

} // Physics3D
} // Impact
