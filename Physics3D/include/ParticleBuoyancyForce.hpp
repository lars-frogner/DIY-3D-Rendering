#pragma once
#include "precision.hpp"
#include "ParticleForceGenerator.hpp"
#include "Particle.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleBuoyancyForce : public ParticleForceGenerator {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	imp_float _particle_volume;
	imp_float _particle_submersion_depth;
	imp_float _surface_height;
	imp_float _liquid_density;

public:

	ParticleBuoyancyForce(imp_float new_particle_volume,
						  imp_float new_particle_submersion_depth,
						  imp_float new_surface_height,
						  imp_float new_liquid_density = 1000);

	virtual void addForce(Particle* particle, imp_float duration);

};

} // Physics3D
} // Impact
