#include "ParticleModel.hpp"

namespace Impact {
namespace Rendering3D {

ParticleModel::ParticleModel(const TriangleMesh* new_mesh,
							 const Material* new_material,
							 Particle* new_particle)
		: Model(new_mesh, new_material),
		  _particle(new_particle) {}

ParticleModel::ParticleModel(const TriangleMesh* new_mesh,
							 const Material* new_material,
							 Particle* new_particle,
							 const AffineTransformation& new_transformation)
		: Model(new_mesh, new_material, new_transformation),
		  _particle(new_particle) {}

Geometry3D::TriangleMesh ParticleModel::getTransformedMesh() const
{
	return TriangleMesh(*_mesh).applyTransformation(_particle->getTransformation()(_transformation));
}

Geometry3D::TriangleMesh ParticleModel::getTransformedMesh(const AffineTransformation& additional_transformation) const
{
	return TriangleMesh(*_mesh).applyTransformation(additional_transformation(_particle->getTransformation()(_transformation)));
}

Physics3D::Particle* ParticleModel::getParticle()
{
	return _particle;
}

} // Rendering3D
} // Impact
