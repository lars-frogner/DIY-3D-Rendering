#pragma once
#include "precision.hpp"
#include "ParticleContactGenerator.hpp"
#include "Particle.hpp"
#include "ParticleContact.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include <vector>

namespace Impact {
namespace Physics3D {

class ParticleLinkedContact : public ParticleContactGenerator {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	Particle* _particle_1;
	Particle* _particle_2;
	
	Vector getCurrentSeparation() const;
	imp_float getCurrentDistance() const;

public:
	std::vector<Particle*> getInvolvedParticles() const;

	ParticleLinkedContact(Particle* new_particle_1,
						  Particle* new_particle_2);
};

} // Physics3D
} // Impact
