#pragma once
#include "CoordinateFrame.hpp"

namespace Impact {
namespace Geometry3D {

CoordinateFrame::CoordinateFrame(const Point&  new_origin,
                                 const Vector& new_basis_1,
                                 const Vector& new_basis_2,
                                 const Vector& new_basis_3)
    : origin(new_origin),
      basis_1(new_basis_1),
      basis_2(new_basis_2),
      basis_3(new_basis_3) {}

Point CoordinateFrame::operator()(imp_float coord_1, imp_float coord_2, imp_float coord_3) const
{
    return origin + basis_1*coord_1 + basis_2*coord_2 + basis_3*coord_3;
}

CoordinateFrame& CoordinateFrame::rotateFromXToY(imp_float angle)
{
    basis_1.rotateFromXToY(angle);
    basis_2.rotateFromXToY(angle);
    basis_3.rotateFromXToY(angle);
    return *this;
}

CoordinateFrame& CoordinateFrame::rotateFromYToZ(imp_float angle)
{
    basis_1.rotateFromYToZ(angle);
    basis_2.rotateFromYToZ(angle);
    basis_3.rotateFromYToZ(angle);
    return *this;
}

CoordinateFrame& CoordinateFrame::rotateFromZToX(imp_float angle)
{
    basis_1.rotateFromZToX(angle);
    basis_2.rotateFromZToX(angle);
    basis_3.rotateFromZToX(angle);
    return *this;
}

CoordinateFrame& CoordinateFrame::makeAxesPerpendicular()
{
    basis_2 = basis_2.getProjectedOnNormalTo(basis_3, basis_1);
    basis_3 = basis_3.getProjectedOnNormalTo(basis_1, basis_2);
    return *this;
}

} // Geometry3D
} // Impact
