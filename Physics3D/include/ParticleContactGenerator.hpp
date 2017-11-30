#pragma once
#include "precision.hpp"
#include "Particle.hpp"
#include "ParticleContact.hpp"
#include <vector>

namespace Impact {
namespace Physics3D {

class ParticleContactGenerator {

public:
	virtual std::vector<Particle*> getInvolvedParticles() const = 0;

	virtual imp_uint generateContacts(imp_uint n_available_contacts,
									  ParticleContact* first_available_contact) const = 0;
};

} // Physics3D
} // Impact
