#include "ParticleGravityForce.hpp"
#include <cassert>
#include <cmath>

namespace Impact {
namespace Physics3D {

ParticleGravityForce::ParticleGravityForce(Particle* new_particle_1,
										   Particle* new_particle_2,
										   imp_float new_gravitational_constant)
	: _particle_1(new_particle_1),
	  _particle_2(new_particle_2),
	  _force_constant(-new_gravitational_constant*new_particle_1->getMass()*new_particle_2->getMass()) {}

void ParticleGravityForce::addForce(Particle* particle, imp_float duration)
{
	assert(particle == _particle_1 || particle == _particle_2);

	if (_has_calculated_force)
	{
		_has_calculated_force = false;
	}
	else
	{
		_has_calculated_force = true;

		_force_from_2_on_1 = _particle_1->getPosition() - _particle_2->getPosition();
		imp_float distance_squared = _force_from_2_on_1.getSquaredLength();

		_force_from_2_on_1 *= _force_constant/(sqrt(distance_squared)*distance_squared);
	}

	particle->addForce((particle == _particle_1)? _force_from_2_on_1 : -_force_from_2_on_1);
}

} // Physics3D
} // Impact
