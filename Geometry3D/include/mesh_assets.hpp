#pragma once
#include "TriangleMesh.hpp"

namespace Impact {
namespace Geometry3D {

#define IMP_N_SPHERE_RESOLUTIONS 6
#define IMP_N_SPHERE_TEXTURE_MODES 3

extern const TriangleMesh IMP_ROOM_MESH;
extern const TriangleMesh IMP_BOX_MESH;
extern const TriangleMesh IMP_SHEET_MESH;
extern const TriangleMesh IMP_TWOSIDED_SHEET_MESH;
extern const TriangleMesh IMP_SPHERE_MESHES[IMP_N_SPHERE_TEXTURE_MODES][IMP_N_SPHERE_RESOLUTIONS];
extern const TriangleMesh IMP_TWOSIDED_SPHERE_MESHES[IMP_N_SPHERE_RESOLUTIONS];
//extern const TriangleMesh IMP_TEAPOT_MESH;

} // Geometry3D
} // Impact
