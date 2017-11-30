#pragma once
#include "precision.hpp"
#include "ParticleForceGenerator.hpp"
#include "Particle.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleGravityForce : public ParticleForceGenerator {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	Vector _force_from_2_on_1;

	Particle* _particle_1;
	Particle* _particle_2;

	imp_float _force_constant;

	bool _has_calculated_force = false;

public:

	ParticleGravityForce(Particle* new_particle_1,
						 Particle* new_particle_2,
						 imp_float new_gravitational_constant);

	virtual void addForce(Particle* particle, imp_float duration);

};

} // Physics3D
} // Impact
