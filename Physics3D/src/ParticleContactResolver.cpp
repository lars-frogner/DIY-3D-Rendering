#include "ParticleContactResolver.hpp"

namespace Impact {
namespace Physics3D {

ParticleContactResolver::ParticleContactResolver(imp_uint new_max_iterations)
	: _max_iterations(new_max_iterations) {}

void ParticleContactResolver::setMaxIterations(imp_uint max_iterations)
{
	_max_iterations = max_iterations;
}

void ParticleContactResolver::resolveCollisions(imp_uint n_contacts,
											    ParticleContact contacts[],
											    imp_float duration)
{
	imp_float most_negative_separation_velocity;
	imp_float separation_velocity;
	imp_uint contact_idx;
	imp_uint idx;

	_velocity_iterations_used = 0;

	while (_velocity_iterations_used < _max_iterations)
	{
		most_negative_separation_velocity = 0;
		contact_idx = n_contacts;

		for (idx = 0; idx < n_contacts; idx++)
		{
			separation_velocity = contacts[idx].getSeparatingVelocity();

			if (separation_velocity < most_negative_separation_velocity)
			{
				most_negative_separation_velocity = separation_velocity;
				contact_idx = idx;
			}
		}

		if (most_negative_separation_velocity >= 0)
			break;

		contacts[contact_idx].resolveCollision(duration);

		_velocity_iterations_used++;
	}
}

void ParticleContactResolver::resolveInterpenetrations(imp_uint n_contacts,
													   ParticleContact contacts[],
													   imp_float duration)
{
	imp_float largest_penetration_depth;
	imp_float penetration_depth;
	imp_uint contact_idx;
	imp_uint idx;

	_interpenetration_iterations_used = 0;

	while (_interpenetration_iterations_used < _max_iterations)
	{
		largest_penetration_depth = 0;
		contact_idx = n_contacts;

		for (idx = 0; idx < n_contacts; idx++)
		{
			contacts[idx].resetLastDisplacements();

			penetration_depth = contacts[idx].getPenetrationDepth();

			if (penetration_depth > largest_penetration_depth)
			{
				largest_penetration_depth = penetration_depth;
				contact_idx = idx;
			}
		}

		if (largest_penetration_depth <= 0)
			break;

		contacts[contact_idx].resolveInterpenetration(duration);
		
		for (idx = 0; idx < n_contacts; idx++)
		{
			contacts[idx].updatePenetrationDepth();
		}

		_interpenetration_iterations_used++;
	}
}

void ParticleContactResolver::resolve(imp_uint n_contacts,
									  ParticleContact contacts[],
									  imp_float duration)
{
	resolveCollisions(n_contacts, contacts, duration);
	resolveInterpenetrations(n_contacts, contacts, duration);
}

} // Physics3D
} // Impact
