#include "mesh_assets.hpp"
#include "Point3.hpp"
#include "Box.hpp"

namespace Impact {
namespace Geometry3D {

extern const TriangleMesh IMP_ROOM_MESH = TriangleMesh::room(Box(Point(-0.5f, 0, -0.5f), 1, 1, 1));
extern const TriangleMesh IMP_BOX_MESH = TriangleMesh::box(Box(Point(-0.5f, -0.5f, -0.5f), 1, 1, 1));
extern const TriangleMesh IMP_SHEET_MESH = TriangleMesh::sheet(Point::origin(), Vector::unitY(), Vector::unitX(), 1.0f);
extern const TriangleMesh IMP_TWOSIDED_SHEET_MESH = TriangleMesh::twoSidedSheet(Point::origin(), Vector::unitY(), Vector::unitX(), 1.0f);

extern const TriangleMesh IMP_SPHERE_MESHES[IMP_N_SPHERE_TEXTURE_MODES][IMP_N_SPHERE_RESOLUTIONS] = 
	{{TriangleMesh::sphere(Sphere(Point::origin(), 1),   8, 0),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1),  16, 0),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1),  32, 0),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1),  64, 0),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1), 128, 0),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1), 256, 0)},
	 {TriangleMesh::sphere(Sphere(Point::origin(), 1),   8, 1),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1),  16, 1),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1),  32, 1),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1),  64, 1),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1), 128, 1),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1), 256, 1)},
	 {TriangleMesh::sphere(Sphere(Point::origin(), 1),   8, 2),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1),  16, 2),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1),  32, 2),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1),  64, 2),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1), 128, 2),
	  TriangleMesh::sphere(Sphere(Point::origin(), 1), 256, 2)}};

extern const TriangleMesh IMP_TWOSIDED_SPHERE_MESHES[IMP_N_SPHERE_RESOLUTIONS] = {TriangleMesh::twoSidedSphere(Sphere(Point::origin(), 1), 8),
																				  TriangleMesh::twoSidedSphere(Sphere(Point::origin(), 1), 16),
																				  TriangleMesh::twoSidedSphere(Sphere(Point::origin(), 1), 32),
																				  TriangleMesh::twoSidedSphere(Sphere(Point::origin(), 1), 64),
																				  TriangleMesh::twoSidedSphere(Sphere(Point::origin(), 1), 128)};

//extern const TriangleMesh IMP_TEAPOT_MESH = TriangleMesh::file("data/teapot.obj");

} // Geometry3D
} // Impact
