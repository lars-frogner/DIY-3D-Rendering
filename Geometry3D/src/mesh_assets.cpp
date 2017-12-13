#include "mesh_assets.hpp"
#include "Point3.hpp"
#include "Box.hpp"

namespace Impact {
namespace Geometry3D {

extern const TriangleMesh IMP_ROOM_MESH = TriangleMesh::room(Box(Point(-0.5f, 0, -0.5f), 1, 1, 1));
extern const TriangleMesh IMP_BOX_MESH = TriangleMesh::box(Box(Point(-0.5f, -0.5f, -0.5f), 1, 1, 1));
extern const TriangleMesh IMP_SHEET_MESH = TriangleMesh::sheet(Point::origin(), Vector::unitY(), Vector::unitX(), 1.0f);
extern const TriangleMesh IMP_TWOSIDED_SHEET_MESH = TriangleMesh::twoSidedSheet(Point::origin(), Vector::unitY(), Vector::unitX(), 1.0f);

extern const TriangleMesh IMP_SPHERE_MESHES[IMP_N_SPHERE_MESHES] = {TriangleMesh::sphere(Sphere(Point::origin(), 1), 8),
																    TriangleMesh::sphere(Sphere(Point::origin(), 1), 16),
																    TriangleMesh::sphere(Sphere(Point::origin(), 1), 32),
																    TriangleMesh::sphere(Sphere(Point::origin(), 1), 64),
																    TriangleMesh::sphere(Sphere(Point::origin(), 1), 128)};

extern const TriangleMesh IMP_TWOSIDED_SPHERE_MESHES[IMP_N_SPHERE_MESHES] = {TriangleMesh::twoSidedSphere(Sphere(Point::origin(), 1), 8),
																			 TriangleMesh::twoSidedSphere(Sphere(Point::origin(), 1), 16),
																			 TriangleMesh::twoSidedSphere(Sphere(Point::origin(), 1), 32),
																			 TriangleMesh::twoSidedSphere(Sphere(Point::origin(), 1), 64),
																			 TriangleMesh::twoSidedSphere(Sphere(Point::origin(), 1), 128)};

//extern const TriangleMesh IMP_TEAPOT_MESH = TriangleMesh::file("data/teapot.obj");

} // Geometry3D
} // Impact
