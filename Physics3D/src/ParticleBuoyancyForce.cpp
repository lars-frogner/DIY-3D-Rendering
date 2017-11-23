#include "ParticleBuoyancyForce.hpp"
#include <cmath>

namespace Impact {
namespace Physics3D {

ParticleBuoyancyForce::ParticleBuoyancyForce(imp_float new_particle_volume,
											 imp_float new_particle_submersion_depth,
											 imp_float new_surface_height,
											 imp_float new_liquid_density /* = 1000 */)
	: _particle_volume(new_particle_volume),
	  _particle_submersion_depth(new_particle_submersion_depth),
	  _surface_height(new_surface_height),
	  _liquid_density(new_liquid_density) {}

void ParticleBuoyancyForce::addForce(Particle* particle, imp_float duration)
{
	imp_float depth = particle->getPosition().y;

	if (depth >= _surface_height + _particle_submersion_depth)
		return;

	Vector force(0, 0, 0);

	if (depth <= _surface_height - _particle_submersion_depth)
	{
		force.y = _liquid_density*_particle_volume;
		particle->addForce(force);
		return;
	}

	force.y = _liquid_density*_particle_volume*
			  (depth - _particle_submersion_depth - _surface_height)/(2*_particle_submersion_depth);
	
	particle->addForce(force);
}

} // Physics3D
} // Impact
