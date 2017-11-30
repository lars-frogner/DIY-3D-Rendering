#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Geometry3D {

class CoordinateFrame {

public:
    Point origin;
    Vector basis_1, basis_2, basis_3;

    CoordinateFrame();

    CoordinateFrame(const Point&  new_origin,
                    const Vector& new_basis_1,
                    const Vector& new_basis_2,
                    const Vector& new_basis_3);

    Point operator()(imp_float coord_1, imp_float coord_2, imp_float coord_3) const;

    CoordinateFrame& rotateFromXToY(imp_float angle);
    CoordinateFrame& rotateFromYToZ(imp_float angle);
    CoordinateFrame& rotateFromZToX(imp_float angle);

    CoordinateFrame& makeAxesPerpendicular();
};

} // Geometry3D
} // Impact
