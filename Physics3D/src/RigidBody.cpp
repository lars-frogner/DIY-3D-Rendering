#include "RigidBody.hpp"
#include <cassert>
#include <cmath>

namespace Impact {
namespace Physics3D {

RigidBody::RigidBody()
	: _position(Point::origin()),
	  _orientation(Quaternion(1, 0, 0, 0)),
	  _velocity(Vector::zero()),
	  _rotation(Vector::zero()),
	  _inverse_mass(1) {}

RigidBody::RigidBody(const Point& new_position,
				     const Quaternion& new_orientation,
				     const Vector& new_velocity,
				     const Vector& new_rotation,
				     imp_float new_mass)
	: _position(new_position),
	  _orientation(new_orientation),
	  _velocity(new_velocity),
	  _rotation(new_rotation),
	  _inverse_mass(1/new_mass) {}

void RigidBody::setPosition(const Point& position)
{
	_position = position;
}

void RigidBody::setOrientation(const Quaternion& orientation)
{
	_orientation = orientation;
}

void RigidBody::setVelocity(const Vector& velocity)
{
	_velocity = velocity;
}

void RigidBody::setRotation(const Vector& rotation)
{
	_rotation = rotation;
}

void RigidBody::setMass(imp_float mass)
{
	_inverse_mass = 1/mass;
}

void RigidBody::setInverseMass(imp_float inverse_mass)
{
	_inverse_mass = inverse_mass;
}

const Geometry3D::Point& RigidBody::getPosition() const
{
	return _position;
}

const Geometry3D::Quaternion& RigidBody::getOrientation() const
{
	return _orientation;
}

const Geometry3D::Vector& RigidBody::getVelocity() const
{
	return _velocity;
}

const Geometry3D::Vector& RigidBody::getRotation() const
{
	return _rotation;
}

imp_float RigidBody::getMass() const
{
	return 1/_inverse_mass;
}

imp_float RigidBody::getInverseMass() const
{
	return _inverse_mass;
}

bool RigidBody::hasInfiniteMass() const
{
	return _inverse_mass == 0;
}

void RigidBody::calculateDerivedData()
{
	_transform = LinearTransformation::rotation(_orientation)*AffineTransformation::translation(_position.toVector());
}

void RigidBody::integrateMotion(imp_float duration)
{
}

} // Physics3D
} // Impact
