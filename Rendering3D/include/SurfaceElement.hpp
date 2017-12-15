#pragma once
#include "precision.hpp"
#include "Point2.hpp"
#include "Vector2.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "TriangleMesh.hpp"
#include "Material.hpp"
#include "Color.hpp"
#include "Model.hpp"

namespace Impact {
namespace Rendering3D {

class SurfaceElement {

private:
	typedef Geometry2D::Point Point2;
	typedef Geometry2D::Vector Vector2;
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::TriangleMesh TriangleMesh;
	typedef Geometry3D::MeshIntersectionData MeshIntersectionData;

protected:
	
	struct Geometric
	{
		Point position;
		Vector normal;
	};
	
	struct Shading
	{
		Point position;
		Vector normal;
		Vector tangent;
		Vector bitangent;
		Point2 texture_coordinate;
		Color color;
	};

	Vector2 _bump_values;

public:
	const Model* model = nullptr;

	Geometric geometric;
	Shading shading;

	void computeTextureColor();
	void computeBumpMappedNormal();
	void computeDisplacementMappedPosition();

	bool evaluateBumpMapping();
};

} // Rendering3D
} // Impact
