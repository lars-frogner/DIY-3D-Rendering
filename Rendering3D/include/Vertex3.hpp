#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Color.hpp"

namespace Impact {
namespace Rendering3D {

class Vertex3 : public Geometry3D::Point {

public:
    Color color;

    Vertex3();
    Vertex3(imp_float new_x, imp_float new_y, imp_float new_z);
    Vertex3(imp_float new_x, imp_float new_y, imp_float new_z,
            const Color& new_color);
    Vertex3(imp_float new_x, imp_float new_y, imp_float new_z,
            imp_float red, imp_float green, imp_float blue);
};

} // Rendering3D
} // Impact
