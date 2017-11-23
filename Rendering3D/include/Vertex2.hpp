#pragma once
#include "precision.hpp"
#include "Point2.hpp"
#include "Color.hpp"

namespace Impact {
namespace Rendering3D {

class Vertex2 : public Geometry2D::Point {

public:
    Color color;

    Vertex2();

    Vertex2(imp_float new_x, imp_float new_y);

    Vertex2(imp_float new_x, imp_float new_y,
            const Color& new_color);

    Vertex2(imp_float new_x, imp_float new_y,
            imp_float red, imp_float green, imp_float blue);
};

} // Rendering3D
} // Impact
