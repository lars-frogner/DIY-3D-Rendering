#pragma once
#include "precision.hpp"
#include "ParticleContactGenerator.hpp"
#include "Particle.hpp"
#include "ParticleContact.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Plane.hpp"
#include <vector>

namespace Impact {
namespace Physics3D {

class ParticlePlaneContact : public ParticleContactGenerator {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::Plane Plane;

protected:
	Particle* _particle;
	Plane _plane;
	imp_float _restitution_coef;

public:
	ParticlePlaneContact(Particle* new_particle,
						 const Plane& new_plane,
						 imp_float new_restitution_coef);
	
	std::vector<Particle*> getInvolvedParticles() const;

	virtual imp_uint generateContacts(imp_uint n_available_contacts,
									  ParticleContact* first_available_contact) const;
};

} // Physics3D
} // Impact
