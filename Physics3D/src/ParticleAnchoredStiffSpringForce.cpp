#include "ParticleAnchoredStiffSpringForce.hpp"
#include <cassert>
#include <cmath>

namespace Impact {
namespace Physics3D {

ParticleAnchoredStiffSpringForce::ParticleAnchoredStiffSpringForce(const Point& new_anchor_point,
																   imp_float new_spring_constant,
																   imp_float new_damping)
	: _anchor_point(new_anchor_point),
	  _spring_constant(new_spring_constant),
	  _damping(new_damping)
{
	assert(4*_spring_constant > _damping*_damping);
	_angular_frequency = sqrt(4*_spring_constant - _damping*_damping)/2;
}

void ParticleAnchoredStiffSpringForce::setAnchor(const Point& anchor_point)
{
	_anchor_point = anchor_point;
}

void ParticleAnchoredStiffSpringForce::addForce(Particle* particle, imp_float duration)
{
	if (particle->hasInfiniteMass())
		return;

	const Vector& displacement = particle->getPosition() - _anchor_point;
	const Vector& correction = (0.5f*_damping*displacement + particle->getVelocity())/_angular_frequency;
	const Vector& target_displacement = (displacement*cos(_angular_frequency*duration) + correction*sin(_angular_frequency*duration))*exp(-0.5f*_damping*duration);
	const Vector& acceleration = ((target_displacement - displacement)/duration - particle->getVelocity())/duration;

	particle->addForce(acceleration*particle->getMass());
}

} // Physics3D
} // Impact
