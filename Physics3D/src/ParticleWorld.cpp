#include "ParticleWorld.hpp"
#include <cassert>
#include <algorithm>
#include <chrono>
#include <iostream>

namespace Impact {
namespace Physics3D {

ParticleWorld::ParticleWorld(imp_uint new_max_contacts,
							 imp_uint max_iterations /* = 0 */)
	: _max_contacts(new_max_contacts),
	  _calculate_iterations(max_iterations == 0),
	  _contacts(new ParticleContact[new_max_contacts]),
	  _contact_resolver(max_iterations) {}

ParticleWorld::~ParticleWorld()
{
	delete _contacts;
}

void ParticleWorld::resetForces()
{
	std::vector<Particle*>::iterator iter = _particles.begin();

	for (; iter != _particles.end(); iter++)
	{
		(*iter)->resetAccumulatedForce();
	}
}

imp_uint ParticleWorld::generateContacts()
{

	imp_uint n_generated_contacts;
	imp_uint n_available_contacts = _max_contacts;
	ParticleContact* first_available_contact = _contacts;

	std::vector<ParticleContactGenerator*>::const_iterator iter = _contact_generators.begin();

	for (; iter != _contact_generators.end(); iter++)
	{
		n_generated_contacts = (*iter)->generateContacts(n_available_contacts,
														 first_available_contact);

		n_available_contacts -= n_generated_contacts;
		first_available_contact += n_generated_contacts;

		if (n_available_contacts <= 0)
		{
			std::cerr << "Warning: max number of generated contacts exceeded (" << _max_contacts << ")" << std::endl;
			break;
		}
	}

	return _max_contacts - n_available_contacts;
}

void ParticleWorld::integrateMotion(imp_float duration)
{
	std::vector<Particle*>::iterator iter = _particles.begin();

	for (; iter != _particles.end(); iter++)
	{
		(*iter)->integrateMotion(duration);
	}
}

void ParticleWorld::addParticle(Particle* particle)
{
	_particles.push_back(particle);
}

void ParticleWorld::removeParticle(Particle* particle)
{
	_particles.erase(std::remove(_particles.begin(), _particles.end(), particle), _particles.end());
}

void ParticleWorld::clearParticles()
{
	_particles.clear();
}

bool ParticleWorld::hasParticle(Particle* particle) const
{
	return std::find(_particles.begin(), _particles.end(), particle) != _particles.end();
}

bool ParticleWorld::hasParticles(const std::vector<Particle*>& particles) const
{
	std::vector<Particle*>::const_iterator iter = particles.begin();

	for (; iter != particles.end(); iter++)
	{
		if (!hasParticle(*iter))
			return false;
	}

	return true;
}

void ParticleWorld::addContactGenerator(ParticleContactGenerator* contact_generator)
{
	assert(hasParticles(contact_generator->getInvolvedParticles()));
	_contact_generators.push_back(contact_generator);
}

void ParticleWorld::removeContactGenerator(ParticleContactGenerator* contact_generator)
{
	_contact_generators.erase(std::remove(_contact_generators.begin(), _contact_generators.end(), contact_generator), _contact_generators.end());
}

void ParticleWorld::clearContactGenerators()
{
	_contact_generators.clear();
}

void ParticleWorld::addForceGenerator(Particle* particle, ParticleForceGenerator* force_generator)
{
	assert(hasParticle(particle));
	_force_registry.addForceGenerator(particle, force_generator);
}

void ParticleWorld::removeForceGenerator(Particle* particle, ParticleForceGenerator* force_generator)
{
	_force_registry.removeForceGenerator(particle, force_generator);
}

void ParticleWorld::clearForceGenerators()
{
	_force_registry.clearForceGenerators();
}

imp_uint ParticleWorld::getNumberOfParticles() const
{
	return static_cast<imp_uint>(_particles.size());
}

void ParticleWorld::performPerFrameInitialization()
{
	resetForces();
}

void ParticleWorld::advance(imp_float duration)
{
	_force_registry.applyForces(duration);

	integrateMotion(duration);

	imp_uint n_generated_contacts = generateContacts();

	if (_calculate_iterations)
		_contact_resolver.setMaxIterations(2*n_generated_contacts);

	_contact_resolver.resolveContacts(n_generated_contacts, _contacts, duration);
}

void ParticleWorld::updateTransformations()
{
	std::vector<Particle*>::iterator iter = _particles.begin();

	for (; iter != _particles.end(); iter++)
	{
		(*iter)->updateTransformation();
	}
}

} // Physics3D
} // Impact
