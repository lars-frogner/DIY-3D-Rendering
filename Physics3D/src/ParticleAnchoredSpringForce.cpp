#include "ParticleAnchoredSpringForce.hpp"
#include <cmath>

namespace Impact {
namespace Physics3D {

ParticleAnchoredSpringForce::ParticleAnchoredSpringForce(const Point& new_anchor_point,
														 imp_float new_spring_constant,
														 imp_float new_rest_length)
	: _anchor_point(new_anchor_point),
	  _spring_constant(new_spring_constant),
	  _rest_length(new_rest_length) {}

void ParticleAnchoredSpringForce::setAnchor(const Point& anchor_point)
{
	_anchor_point = anchor_point;
}

void ParticleAnchoredSpringForce::addForce(Particle* particle, imp_float duration)
{
	Vector vector = particle->getPosition() - _anchor_point;

	imp_float force = -_spring_constant*abs(vector.getLength() - _rest_length);

	vector.normalize();
	vector *= force;

	particle->addForce(vector);
}

} // Physics3D
} // Impact
