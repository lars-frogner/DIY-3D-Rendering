#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Ray.hpp"
#include "TriangleMesh.hpp"
#include "AffineTransformation.hpp"
#include "Material.hpp"
#include "Color.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class RenderableObject {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::Ray Ray;
	typedef Geometry3D::TriangleMesh TriangleMesh;
	typedef Geometry3D::AffineTransformation AffineTransformation;

protected:
    const TriangleMesh* _mesh;
    const Material* _material;
	AffineTransformation _transformation;
    
    //static std::vector<BlinnPhongMaterial> SceneObject::_getMtlFileData(const std::string& filename);

public:
    bool uses_direct_lighting = true;
	bool casts_shadows = false;
	
    bool render_faces = true;
    bool render_edges = false;

    bool remove_hidden_faces = true;
    bool perform_clipping = true;
	
	bool is_visible = true;
	bool is_currently_visible = is_visible;

    RenderableObject(const TriangleMesh* new_mesh,
					 const Material* new_material,
					 const AffineTransformation& new_transformation);

    RenderableObject(const TriangleMesh* new_mesh,
					 const Material* new_material);

	virtual TriangleMesh getTransformedMesh() const;

	const TriangleMesh* getMesh() const;

	const Material* getMaterial() const;
};

} // Rendering3D
} // Impact
