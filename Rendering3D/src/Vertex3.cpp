#include "Vertex3.hpp"

namespace Impact {
namespace Rendering3D {

Vertex3::Vertex3() : Geometry3D::Point(), color() {}

Vertex3::Vertex3(imp_float new_x, imp_float new_y, imp_float new_z)
    : Geometry3D::Point(new_x, new_y, new_z), color() {}

Vertex3::Vertex3(imp_float new_x, imp_float new_y, imp_float new_z,
               const Color& new_color)
    : Geometry3D::Point(new_x, new_y, new_z), color(new_color) {}

Vertex3::Vertex3(imp_float new_x, imp_float new_y, imp_float new_z,
               imp_float red, imp_float green, imp_float blue)
    : Geometry3D::Point(new_x, new_y, new_z), color(red, green, blue) {}

} // Rendering3D
} // Impact
