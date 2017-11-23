#pragma once
#include "precision.hpp"
#include "ParticleForceGenerator.hpp"
#include "Particle.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleAnchoredStiffSpringForce : public ParticleForceGenerator {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	Point _anchor_point;

	imp_float _spring_constant;
	imp_float _damping;

	imp_float _angular_frequency;

public:

	ParticleAnchoredStiffSpringForce(const Point& new_anchor_point,
									 imp_float new_spring_constant,
									 imp_float new_damping);

	void setAnchor(const Point& anchor_point);

	virtual void addForce(Particle* particle, imp_float duration);

};

} // Physics3D
} // Impact
