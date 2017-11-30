#pragma once
#include "precision.hpp"
#include "RenderableObject.hpp"
#include "Material.hpp"
#include "Particle.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "TriangleMesh.hpp"
#include "AffineTransformation.hpp"

namespace Impact {
namespace Rendering3D {

class RenderableParticle : public RenderableObject {
    
private:
    typedef Physics3D::Particle Particle;
    typedef Geometry3D::Point Point;
    typedef Geometry3D::Vector Vector;
    typedef Geometry3D::TriangleMesh TriangleMesh;
    typedef Geometry3D::AffineTransformation AffineTransformation;

protected:
	Particle* _particle;

public:

    RenderableParticle(const TriangleMesh* new_mesh,
					   const Material* new_material,
					   Particle* new_particle,
					   const AffineTransformation& new_transformation);

    RenderableParticle(const TriangleMesh* new_mesh,
					   const Material* new_material,
					   Particle* new_particle);

	virtual TriangleMesh getTransformedMesh() const;

	Particle* getParticle();
};

} // Rendering3D
} // Impact
