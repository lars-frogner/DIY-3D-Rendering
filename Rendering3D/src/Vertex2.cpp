#include "Vertex2.hpp"

namespace Impact {
namespace Rendering3D {

Vertex2::Vertex2() : Geometry2D::Point(), color() {}

Vertex2::Vertex2(imp_float new_x, imp_float new_y)
    : Geometry2D::Point(new_x, new_y), color() {}

Vertex2::Vertex2(imp_float new_x, imp_float new_y,
                 const Color& new_color)
    : Geometry2D::Point(new_x, new_y), color(new_color) {}

Vertex2::Vertex2(imp_float new_x, imp_float new_y,
                 imp_float red, imp_float green, imp_float blue)
    : Geometry2D::Point(new_x, new_y), color(red, green, blue) {}

} // Rendering3D
} // Impact
