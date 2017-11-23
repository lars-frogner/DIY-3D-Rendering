#pragma once
#include "precision.hpp"
#include "Particle.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleContact {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	Particle* _particle_1;
	Particle* _particle_2;
	Vector _contact_normal;
	imp_float _penetration_depth;
	imp_float _restitution_coef;

public:
	ParticleContact();

	ParticleContact(Particle* new_particle_1,
					Particle* new_particle_2,
					const Vector& new_contact_normal,
					imp_float new_penetration_depth,
					imp_float new_restitution_coef);

	void setParticles(Particle* particle_1, Particle* particle_2);
	void setContactNormal(const Vector& contact_normal);
	void setPenetrationDepth(imp_float penetration_depth);
	void setRestitutionCoefficient(imp_float restitution_coef);

	imp_float getSeparatingVelocity() const;
	imp_float getPenetrationDepth() const;
	
	void updatePenetrationDepth();
	void resetLastDisplacements();

	void resolveCollision(imp_float duration);
	void resolveInterpenetration(imp_float duration);
};

} // Physics3D
} // Impact
