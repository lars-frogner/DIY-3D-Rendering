#include "RenderableParticle.hpp"

namespace Impact {
namespace Rendering3D {

RenderableParticle::RenderableParticle(const TriangleMesh* new_mesh,
									   const Material* new_material,
									   Particle* new_particle)
		: RenderableObject(new_mesh, new_material),
		  _particle(new_particle) {}

RenderableParticle::RenderableParticle(const TriangleMesh* new_mesh,
									   const Material* new_material,
									   Particle* new_particle,
									   const AffineTransformation& new_transformation)
		: RenderableObject(new_mesh, new_material, new_transformation),
		  _particle(new_particle) {}

Geometry3D::TriangleMesh RenderableParticle::getTransformedMesh() const
{
	return TriangleMesh(*_mesh).applyTransformation(_particle->getTransformation()*_transformation);
}

Physics3D::Particle* RenderableParticle::getParticle()
{
	return _particle;
}

} // Rendering3D
} // Impact
