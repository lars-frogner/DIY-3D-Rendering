#include "Particle.hpp"
#include <cassert>
#include <cmath>

namespace Impact {
namespace Physics3D {

Particle::Particle()
	: _position(Point::origin()),
	  _velocity(Vector::zero()),
	  _default_acceleration(Vector::zero()),
	  _accumulated_force(Vector::zero()),
	  _inverse_mass(1),
	  _radius(1),
	  _damping(1) {}

Particle::Particle(const Point& new_position,
				   const Vector& new_velocity,
				   imp_float new_mass,
				   imp_float new_radius,
				   imp_float new_damping /* = 1 */)
	: _position(new_position),
	  _velocity(new_velocity),
	  _default_acceleration(Vector::zero()),
	  _accumulated_force(Vector::zero()),
	  _transformation(AffineTransformation::translationTo(new_position)),
	  _inverse_mass(1/new_mass),
	  _radius(new_radius),
	  _damping(new_damping) {}

Particle::Particle(const Point& new_position,
				   const Vector& new_velocity,
				   const Vector& new_default_acceleration,
				   imp_float new_mass,
				   imp_float new_radius,
				   imp_float new_damping /* = 1 */)
	: _position(new_position),
	  _velocity(new_velocity),
	  _default_acceleration(new_default_acceleration),
	  _accumulated_force(Vector::zero()),
	  _transformation(AffineTransformation::translationTo(new_position)),
	  _inverse_mass(1/new_mass),
	  _radius(new_radius),
	  _damping(new_damping) {}

void Particle::setPosition(const Point& position)
{
	_position = position;
}

void Particle::setVelocity(const Vector& velocity)
{
	_velocity = velocity;
}

void Particle::setDefaultAcceleration(const Vector& default_acceleration)
{
	_default_acceleration = default_acceleration;
}

void Particle::setMass(imp_float mass)
{
	_inverse_mass = 1/mass;
}

void Particle::setInverseMass(imp_float inverse_mass)
{
	_inverse_mass = inverse_mass;
}

void Particle::setRadius(imp_float radius)
{
	_radius = radius;
}

void Particle::setDamping(imp_float damping)
{
	_damping = damping;
}

const Geometry3D::Point& Particle::getPosition() const
{
	return _position;
}

const Geometry3D::Vector& Particle::getVelocity() const
{
	return _velocity;
}

const Geometry3D::Vector& Particle::getDefaultAcceleration() const
{
	return _default_acceleration;
}

Geometry3D::Vector Particle::getTotalAcceleration() const
{
	return _default_acceleration + _inverse_mass*_accumulated_force;
}

const Geometry3D::AffineTransformation& Particle::getTransformation() const
{
	return _transformation;
}

imp_float Particle::getMass() const
{
	return 1/_inverse_mass;
}

imp_float Particle::getInverseMass() const
{
	return _inverse_mass;
}

imp_float Particle::getRadius() const
{
	return _radius;
}

imp_float Particle::getDamping() const
{
	return _damping;
}

bool Particle::hasInfiniteMass() const
{
	return _inverse_mass == 0;
}

void Particle::addForce(const Vector& force)
{
	_accumulated_force += force;
}

void Particle::addImpulse(const Vector& impulse)
{
	_velocity += impulse*_inverse_mass;
}

void Particle::addDisplacementImpulse(const Vector& displacement_impulse)
{
	assert(!_was_displaced);
	_last_displacement = displacement_impulse*_inverse_mass;
	_was_displaced = true;
	_position += _last_displacement;
}

bool Particle::wasDisplaced() const
{
	return _was_displaced;
}

const Geometry3D::Vector& Particle::getLastDisplacement() const
{
	return _last_displacement;
}

void Particle::resetLastDisplacement()
{
	if (_was_displaced)
	{
		_last_displacement.setToZero();
		_was_displaced = false;
	}
}

void Particle::resetAccumulatedForce()
{
	_accumulated_force.setToZero();
}

void Particle::integrateMotion(imp_float duration)
{
	assert(duration > 0);

	Vector total_acceleration = _default_acceleration;
	total_acceleration.addScaledVector(_accumulated_force, _inverse_mass);

	_velocity.addScaledVector(total_acceleration, duration);

	if (_damping != 1)
	{
		_velocity *= pow(_damping, duration);
	}

	_position.addScaledVector(_velocity, duration);
}

void Particle::updateTransformation()
{
	_transformation = AffineTransformation::translationTo(_position);
}

} // Physics3D
} // Impact
