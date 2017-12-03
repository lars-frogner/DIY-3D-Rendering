#include "ParticleContactResolver.hpp"

namespace Impact {
namespace Physics3D {

ParticleContactResolver::ParticleContactResolver(imp_uint new_max_iterations)
	: _max_iterations(new_max_iterations) {}

void ParticleContactResolver::setMaxIterations(imp_uint max_iterations)
{
	_max_iterations = max_iterations;
}

void ParticleContactResolver::resolveCollisions(std::vector<ParticleContact>& contacts,
											    imp_float duration)
{
	imp_float global_most_negative_separation_velocity, most_negative_separation_velocity;
	imp_float separation_velocity;
	int idx, global_contact_idx, contact_idx;
	int n_contacts = static_cast<imp_uint>(contacts.size());

	_velocity_iterations_used = 0;

	#pragma omp parallel default(shared) \
						 private(idx, contact_idx, separation_velocity, most_negative_separation_velocity) \
						 shared(contacts, n_contacts, global_contact_idx, global_most_negative_separation_velocity, duration) \
						 if (use_omp)
	{
		while (_velocity_iterations_used < _max_iterations)
		{
			most_negative_separation_velocity = 0;

			#pragma omp for schedule(static)
			for (idx = 0; idx < n_contacts; idx++)
			{
				separation_velocity = contacts[idx].getSeparatingVelocity();

				if (separation_velocity < most_negative_separation_velocity)
				{
					most_negative_separation_velocity = separation_velocity;
					contact_idx = idx;
				}
			}

			#pragma omp single
			global_most_negative_separation_velocity = 0;

			#pragma omp critical
			if (most_negative_separation_velocity < global_most_negative_separation_velocity)
			{
				global_most_negative_separation_velocity = most_negative_separation_velocity;
				global_contact_idx = contact_idx;
			}

			if (global_most_negative_separation_velocity >= 0)
				break;
			
			#pragma omp single
			{
				contacts[contact_idx].resolveCollision(duration);

				_velocity_iterations_used++;
			}
		}
	}
}

void ParticleContactResolver::resolveInterpenetrations(std::vector<ParticleContact>& contacts,
													   imp_float duration)
{
	imp_float global_largest_penetration_depth, largest_penetration_depth;
	imp_float penetration_depth;
	int idx, global_contact_idx, contact_idx;
	int n_contacts = static_cast<imp_uint>(contacts.size());

	_interpenetration_iterations_used = 0;
	
	#pragma omp parallel default(shared) \
						 private(idx, contact_idx, penetration_depth, largest_penetration_depth) \
						 shared(contacts, n_contacts, global_contact_idx, global_largest_penetration_depth, duration) \
						 if (use_omp)
	{
		while (_interpenetration_iterations_used < _max_iterations)
		{
			largest_penetration_depth = 0;
			
			#pragma omp for schedule(static)
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

			#pragma omp single
			global_largest_penetration_depth = 0;

			#pragma omp critical
			if (largest_penetration_depth > global_largest_penetration_depth)
			{
				global_largest_penetration_depth = largest_penetration_depth;
				global_contact_idx = contact_idx;
			}

			if (largest_penetration_depth <= 0)
				break;

			#pragma omp single
			contacts[contact_idx].resolveInterpenetration(duration);
		
			#pragma omp for schedule(static)
			for (idx = 0; idx < n_contacts; idx++)
			{
				contacts[idx].updatePenetrationDepth();
			}
			
			#pragma omp single
			_interpenetration_iterations_used++;
		}
	}
}

void ParticleContactResolver::resolveContacts(std::vector<ParticleContact>& contacts,
											  imp_float duration)
{
	resolveCollisions(contacts, duration);
	resolveInterpenetrations(contacts, duration);
}

} // Physics3D
} // Impact
