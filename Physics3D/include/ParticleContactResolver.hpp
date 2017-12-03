#pragma once
#include "precision.hpp"
#include "ParticleContact.hpp"
#include <vector>

namespace Impact {
namespace Physics3D {

class ParticleContactResolver {

protected:
	imp_uint _max_iterations;
	imp_uint _velocity_iterations_used;
	imp_uint _interpenetration_iterations_used;

	void resolveCollisions(std::vector<ParticleContact>& contacts,
						   imp_float duration);

	void resolveInterpenetrations(std::vector<ParticleContact>& contacts,
								  imp_float duration);

public:
	bool use_omp = true;

	ParticleContactResolver(imp_uint new_max_iterations);

	void setMaxIterations(imp_uint max_iterations);

	void resolveContacts(std::vector<ParticleContact>& contacts,
						 imp_float duration);
};

} // Physics3D
} // Impact
