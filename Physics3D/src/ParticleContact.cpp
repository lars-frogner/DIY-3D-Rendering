#include "ParticleContact.hpp"

namespace Impact {
namespace Physics3D {

ParticleContact::ParticleContact()
	: _particle_1(nullptr),
	  _particle_2(nullptr),
	  _contact_normal(Vector::zero()),
	  _penetration_depth(0),
	  _restitution_coef(1) {}

ParticleContact::ParticleContact(Particle* new_particle_1,
								 Particle* new_particle_2,
								 const Vector& new_contact_normal,
								 imp_float new_penetration_depth,
								 imp_float new_restitution_coef)
	: _particle_1(new_particle_1),
	  _particle_2(new_particle_2),
	  _contact_normal(new_contact_normal),
	  _penetration_depth(new_penetration_depth),
	  _restitution_coef(new_restitution_coef) {}

void ParticleContact::setParticles(Particle* particle_1, Particle* particle_2)
{
	_particle_1 = particle_1;
	_particle_2 = particle_2;
}

void ParticleContact::setContactNormal(const Vector& contact_normal)
{
	_contact_normal = contact_normal;
}

void ParticleContact::setPenetrationDepth(imp_float penetration_depth)
{
	_penetration_depth = penetration_depth;
}

void ParticleContact::setRestitutionCoefficient(imp_float restitution_coef)
{
	_restitution_coef = restitution_coef;
}

imp_float ParticleContact::getSeparatingVelocity() const
{
	Vector relative_velocity(_particle_1->getVelocity());

	if (_particle_2)
		relative_velocity -= _particle_2->getVelocity();

	return relative_velocity.dot(_contact_normal);
}

imp_float ParticleContact::getPenetrationDepth() const
{
	return _penetration_depth;
}

void ParticleContact::updatePenetrationDepth()
{
	if (_particle_1->wasDisplaced())
	{
		Vector relative_displacement = _particle_1->getLastDisplacement();
		
		if (_particle_2 && _particle_2->wasDisplaced())
		{
			relative_displacement -= _particle_2->getLastDisplacement();
		}

		_penetration_depth -= relative_displacement.dot(_contact_normal);
	}
	else if (_particle_2 && _particle_2->wasDisplaced())
	{
		_penetration_depth += (_particle_2->getLastDisplacement()).dot(_contact_normal);
	}
}

void ParticleContact::resetLastDisplacements()
{
	_particle_1->resetLastDisplacement();

	if (_particle_2)
		_particle_2->resetLastDisplacement();
}

void ParticleContact::resolveCollision(imp_float duration)
{
	imp_float separating_velocity = getSeparatingVelocity();

	if (separating_velocity > 0)
		return;

	imp_float new_separating_velocity = -_restitution_coef*separating_velocity;

	Vector relative_acceleration = _particle_1->getTotalAcceleration();

	if (_particle_2)
		relative_acceleration -= _particle_2->getTotalAcceleration();

	imp_float rel_sep_velocity_from_acceleration = duration*(relative_acceleration.dot(_contact_normal));

	if (rel_sep_velocity_from_acceleration < 0)
	{
		// Remove component of separating velocity caused by build-up from acceleration during the frame
		new_separating_velocity += _restitution_coef*rel_sep_velocity_from_acceleration;

		if (new_separating_velocity < 0)
			new_separating_velocity = 0;
	}

	imp_float total_inverse_mass = _particle_1->getInverseMass();

	if (_particle_2)
		total_inverse_mass += _particle_2->getInverseMass();

	if (total_inverse_mass <= 0)
		return;

	Vector impulse = ((new_separating_velocity - separating_velocity)/total_inverse_mass)*_contact_normal;

	_particle_1->addImpulse(impulse);
	
	if (_particle_2)
		_particle_2->addImpulse(-impulse);
}

void ParticleContact::resolveInterpenetration(imp_float duration)
{
	if (_penetration_depth <= 0)
		return;
	
	imp_float total_inverse_mass = _particle_1->getInverseMass();

	if (_particle_2)
		total_inverse_mass += _particle_2->getInverseMass();
	
	if (total_inverse_mass <= 0)
		return;

	Vector position_impulse = (_penetration_depth/total_inverse_mass)*_contact_normal;
	
	_particle_1->addDisplacementImpulse(position_impulse);
	
	if (_particle_2)
		_particle_2->addDisplacementImpulse(-position_impulse);
}

} // Physics3D
} // Impact
