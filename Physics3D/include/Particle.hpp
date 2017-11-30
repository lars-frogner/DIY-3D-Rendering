#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "AffineTransformation.hpp"

namespace Impact {
namespace Physics3D {

class Particle {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::AffineTransformation AffineTransformation;

protected:
	Point _position;
	Vector _velocity;
	Vector _default_acceleration;
	Vector _accumulated_force;
	Vector _last_displacement;
	
	AffineTransformation _transformation;

	imp_float _inverse_mass;
	imp_float _radius;
	imp_float _damping;

	bool _was_displaced = false;

public:

	Particle();

	Particle(const Point& new_position,
			 const Vector& new_velocity,
			 imp_float new_mass,
			 imp_float new_radius,
			 imp_float new_damping = 1);

	Particle(const Point& new_position,
			 const Vector& new_velocity,
			 const Vector& new_default_acceleration,
			 imp_float new_mass,
			 imp_float new_radius,
			 imp_float new_damping = 1);
	
	void setPosition(const Point& position);
	void setVelocity(const Vector& velocity);
	void setDefaultAcceleration(const Vector& default_acceleration);
	void setMass(imp_float mass);
	void setInverseMass(imp_float inverse_mass);
	void setRadius(imp_float radius);
	void setDamping(imp_float damping);
	
	const Point& getPosition() const;
	const Vector& getVelocity() const;
	const Vector& getDefaultAcceleration() const;
	Vector Particle::getTotalAcceleration() const;
	const AffineTransformation& getTransformation() const;
	imp_float getMass() const;
	imp_float getInverseMass() const;
	imp_float getRadius() const;
	imp_float getDamping() const;

	bool hasInfiniteMass() const;

	void addForce(const Vector& force);
	void addImpulse(const Vector& impulse);

	void addDisplacementImpulse(const Vector& displacement_impulse);
	bool wasDisplaced() const;
	const Vector& getLastDisplacement() const;
	void resetLastDisplacement();

	void resetAccumulatedForce();

	void integrateMotion(imp_float duration);

	void updateTransformation();
};

} // Physics3D
} // Impact
