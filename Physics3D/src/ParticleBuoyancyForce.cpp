#include "ParticleBuoyancyForce.hpp"
#include <cmath>

namespace Impact {
namespace Physics3D {

ParticleBuoyancyForce::ParticleBuoyancyForce(imp_float new_surface_height,
											 imp_float new_liquid_density /* = 1000 */)
	: _surface_height(new_surface_height),
	  _liquid_density(new_liquid_density) {}

void ParticleBuoyancyForce::addForce(Particle* particle, imp_float duration)
{
	imp_float depth = particle->getPosition().y;
	imp_float particle_submersion_depth = 2*particle->getRadius();
	
	if (depth >= _surface_height + particle_submersion_depth)
		return;

	imp_float particle_volume = IMP_PI*particle_submersion_depth*particle_submersion_depth/3;

	Vector force(0, 0, 0);

	if (depth <= _surface_height - particle_submersion_depth)
	{
		force.y = _liquid_density*particle_volume;
		particle->addForce(force);
		return;
	}

	force.y = _liquid_density*particle_volume*
			  (depth - particle_submersion_depth - _surface_height)/(2*particle_submersion_depth);
	
	particle->addForce(force);
}

} // Physics3D
} // Impact
