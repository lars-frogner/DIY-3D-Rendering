#pragma once
#include "precision.hpp"
#include "Model.hpp"
#include "Material.hpp"
#include "Particle.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "TriangleMesh.hpp"
#include "AffineTransformation.hpp"

namespace Impact {
namespace Rendering3D {

class ParticleModel : public Model {
    
private:
    typedef Physics3D::Particle Particle;
    typedef Geometry3D::Point Point;
    typedef Geometry3D::Vector Vector;
    typedef Geometry3D::TriangleMesh TriangleMesh;
    typedef Geometry3D::AffineTransformation AffineTransformation;

protected:
	Particle* _particle;

public:

    ParticleModel(const TriangleMesh* new_mesh,
				  const Material* new_material,
			      Particle* new_particle,
		   	      const AffineTransformation& new_transformation);

    ParticleModel(const TriangleMesh* new_mesh,
				  const Material* new_material,
				  Particle* new_particle);

	TriangleMesh getTransformedMesh() const;
	TriangleMesh getTransformedMesh(const AffineTransformation& additional_transformation) const;

	Particle* getParticle();
};

} // Rendering3D
} // Impact
