#include "ParticleWorld.hpp"
#include <cassert>
#include <algorithm>
#include <chrono>
#include <iostream>

namespace Impact {
namespace Physics3D {

ParticleWorld::ParticleWorld(imp_uint max_iterations /* = 0 */)
	: _calculate_iterations(max_iterations == 0),
	  _contact_resolver(max_iterations) {}

void ParticleWorld::resetForces()
{
	int idx;
	int n_particles = static_cast<int>(_particles.size());
	
	#pragma omp parallel for default(shared) \
							 private(idx) \
							 shared(n_particles) \
							 schedule(static) \
							 if (use_omp)
	for (idx = 0; idx < n_particles; idx++)
	{
		_particles[idx]->resetAccumulatedForce();
	}
}

void ParticleWorld::generateContacts()
{
	int idx;
	int n_contact_generators = static_cast<int>(_contact_generators.size());
	std::list<ParticleContact> contact_list, contact_sublist;

	#pragma omp parallel default(shared) \
						 private(idx, contact_sublist) \
						 shared(n_contact_generators, contact_list) \
						 if (use_omp)
	{
		#pragma omp for schedule(dynamic)
		for (idx = 0; idx < n_contact_generators; idx++)
		{
			_contact_generators[idx]->generateContacts(contact_sublist);
		}

		#pragma omp critical
		contact_list.splice(contact_list.end(), contact_sublist);
	}

	_contacts.assign(contact_list.begin(), contact_list.end());
}

void ParticleWorld::integrateMotion(imp_float duration)
{
	int idx;
	int n_particles = static_cast<int>(_particles.size());
	
	#pragma omp parallel for default(shared) \
							 private(idx) \
							 shared(n_particles, duration) \
							 schedule(static) \
							 if (use_omp)
	for (idx = 0; idx < n_particles; idx++)
	{
		_particles[idx]->integrateMotion(duration);
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

void ParticleWorld::initialize()
{
}

void ParticleWorld::performPerFrameInitialization()
{
	resetForces();
	_contact_resolver.use_omp = use_omp;
}

void ParticleWorld::advance(imp_float duration)
{
	_force_registry.applyForces(duration);

	integrateMotion(duration);

	generateContacts();

	if (_calculate_iterations)
		_contact_resolver.setMaxIterations(2*static_cast<imp_uint>(_contacts.size()));

	_contact_resolver.resolveContacts(_contacts, duration);
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
