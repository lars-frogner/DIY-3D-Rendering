#include "ParticleDragForce.hpp"
#include <cmath>

namespace Impact {
namespace Physics3D {

ParticleDragForce::ParticleDragForce(imp_float new_linear_drag_coef, imp_float new_nonlinear_drag_coef)
	: _linear_drag_coef(new_linear_drag_coef),
	  _nonlinear_drag_coef(new_nonlinear_drag_coef) {}

void ParticleDragForce::addForce(Particle* particle, imp_float duration)
{
	Vector vector(particle->getVelocity());

	imp_float squared_speed = vector.getSquaredLength();
	imp_float drag = -_linear_drag_coef*sqrt(squared_speed) - _nonlinear_drag_coef*squared_speed;

	vector.normalize();
	vector *= drag;

	particle->addForce(vector);
}

} // Physics3D
} // Impact
