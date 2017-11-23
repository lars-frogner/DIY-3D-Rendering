#pragma once
#include "precision.hpp"
#include "ParticleForceGenerator.hpp"
#include "Particle.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Physics3D {

class ParticleAnchoredBungeeForce : public ParticleForceGenerator {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	Point _anchor_point;

	imp_float _spring_constant;
	imp_float _rest_length;

public:

	ParticleAnchoredBungeeForce(const Point& new_anchor_point,
							    imp_float new_spring_constant,
							    imp_float new_rest_length);

	void setAnchor(const Point& anchor_point);

	virtual void addForce(Particle* particle, imp_float duration);

};

} // Physics3D
} // Impact
