#pragma once
#include "precision.hpp"
#include "ParticleForceGenerator.hpp"
#include "Particle.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleDragForce : public ParticleForceGenerator {

private:
	typedef Geometry3D::Vector Vector;

protected:
	imp_float _linear_drag_coef;
	imp_float _nonlinear_drag_coef;

public:

	ParticleDragForce(imp_float new_linear_drag_coef, imp_float new_nonlinear_drag_coef);

	virtual void addForce(Particle* particle, imp_float duration);

};

} // Physics3D
} // Impact
