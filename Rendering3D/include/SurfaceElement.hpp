#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Material.hpp"
#include "Color.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class SurfaceElement {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

protected:
	
	struct SurfaceData
	{
		Point position;
		Vector normal;
	};

public:
	const Material* material = nullptr;

	SurfaceData geometric;
	SurfaceData shading;

	imp_uint model_id;
};

} // Rendering3D
} // Impact
