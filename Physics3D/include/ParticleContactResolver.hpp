#pragma once
#include "precision.hpp"
#include "ParticleContact.hpp"

namespace Impact {
namespace Physics3D {

class ParticleContactResolver {

protected:
	imp_uint _max_iterations;
	imp_uint _velocity_iterations_used;
	imp_uint _interpenetration_iterations_used;

	void resolveCollisions(imp_uint n_contacts,
						   ParticleContact contacts[],
						   imp_float duration);

	void resolveInterpenetrations(imp_uint n_contacts,
								  ParticleContact contacts[],
								  imp_float duration);

public:
	ParticleContactResolver(imp_uint new_max_iterations);

	void setMaxIterations(imp_uint max_iterations);

	void resolveContacts(imp_uint n_contacts,
						 ParticleContact contacts[],
						 imp_float duration);
};

} // Physics3D
} // Impact
