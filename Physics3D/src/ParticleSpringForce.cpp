#include "ParticleSpringForce.hpp"
#include <cassert>
#include <cmath>

namespace Impact {
namespace Physics3D {

ParticleSpringForce::ParticleSpringForce(Particle* new_particle_1,
										 Particle* new_particle_2,
										 imp_float new_spring_constant,
										 imp_float new_rest_length)
	: _particle_1(new_particle_1),
	  _particle_2(new_particle_2),
	  _spring_constant(new_spring_constant),
	  _rest_length(new_rest_length) {}

void ParticleSpringForce::addForce(Particle* particle, imp_float duration)
{
	assert(particle == _particle_1 || particle == _particle_2);

	if (_has_calculated_force)
	{
		_has_calculated_force = false;
	}
	else
	{
		_force_from_2_on_1 = _particle_1->getPosition() - _particle_2->getPosition();
		imp_float force = -_spring_constant*abs(_force_from_2_on_1.getLength() - _rest_length);

		_force_from_2_on_1.normalize();
		_force_from_2_on_1 *= force;

		_has_calculated_force = true;
	}

	particle->addForce((particle == _particle_1)? _force_from_2_on_1 : -_force_from_2_on_1);
}

} // Physics3D
} // Impact
