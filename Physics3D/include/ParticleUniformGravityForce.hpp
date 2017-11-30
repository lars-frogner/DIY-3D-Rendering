#pragma once
#include "precision.hpp"
#include "ParticleForceGenerator.hpp"
#include "Particle.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleUniformGravityForce : public ParticleForceGenerator {

private:
	typedef Geometry3D::Vector Vector;

protected:
	Vector _gravitational_acceleration;

public:

	ParticleUniformGravityForce(const Vector& new_gravitational_acceleration);

	virtual void addForce(Particle* particle, imp_float duration);

};

} // Physics3D
} // Impact
