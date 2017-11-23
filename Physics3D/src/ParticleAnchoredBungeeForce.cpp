#include "ParticleAnchoredBungeeForce.hpp"
#include <cmath>

namespace Impact {
namespace Physics3D {

ParticleAnchoredBungeeForce::ParticleAnchoredBungeeForce(const Point& new_anchor_point,
														 imp_float new_spring_constant,
														 imp_float new_rest_length)
	: _anchor_point(new_anchor_point),
	  _spring_constant(new_spring_constant),
	  _rest_length(new_rest_length) {}

void ParticleAnchoredBungeeForce::setAnchor(const Point& anchor_point)
{
	_anchor_point = anchor_point;
}

void ParticleAnchoredBungeeForce::addForce(Particle* particle, imp_float duration)
{
	Vector vector = particle->getPosition() - _anchor_point;
	imp_float length = vector.getLength();

	if (length <= _rest_length)
		return;

	vector.normalize();
	vector *= _spring_constant*(_rest_length - length);

	particle->addForce(vector);
}

} // Physics3D
} // Impact
