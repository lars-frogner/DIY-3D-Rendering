#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"

namespace Impact {
namespace Physics3D {

class RigidBody {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::Quaternion Quaternion;
	typedef Geometry3D::LinearTransformation LinearTransformation;
	typedef Geometry3D::AffineTransformation AffineTransformation;

protected:
	Point _position;
	Quaternion _orientation;
	Vector _velocity;
	Vector _rotation;
	AffineTransformation _transform;
	imp_float _inverse_mass;

public:

	RigidBody();

	RigidBody(const Point& new_position,
			  const Quaternion& new_orientation,
			  const Vector& new_velocity,
			  const Vector& new_rotation,
			  imp_float new_mass);
	
	void setPosition(const Point& position);
	void setOrientation(const Quaternion& orientation);
	void setVelocity(const Vector& velocity);
	void setRotation(const Vector& rotation);
	void setMass(imp_float mass);
	void setInverseMass(imp_float inverse_mass);
	
	const Point& getPosition() const;
	const Quaternion& getOrientation() const;
	const Vector& getVelocity() const;
	const Vector& getRotation() const;
	imp_float getMass() const;
	imp_float getInverseMass() const;

	bool hasInfiniteMass() const;

	void calculateDerivedData();

	void integrateMotion(imp_float duration);
};

} // Physics3D
} // Impact
