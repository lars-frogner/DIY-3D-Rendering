#pragma once
#include "Point.hpp"
#include "Vector.hpp"

namespace Geometry3D {

template <typename F>
class CoordinateFrame {

public:
    Point<F> origin;
    Vector<F> basis_1, basis_2, basis_3;

    CoordinateFrame<F>(const Point<F>&  new_origin,
                       const Vector<F>& new_basis_1,
                       const Vector<F>& new_basis_2,
                       const Vector<F>& new_basis_3);

    Point<F> operator()(F coord_1, F coord_2, F coord_3) const;

    CoordinateFrame<F>& rotateFromXToY(F angle);
    CoordinateFrame<F>& rotateFromYToZ(F angle);
    CoordinateFrame<F>& rotateFromZToX(F angle);

    CoordinateFrame<F>& makeAxesPerpendicular();
};

template <typename F>
CoordinateFrame<F>::CoordinateFrame(const Point<F>&  new_origin,
                                    const Vector<F>& new_basis_1,
                                    const Vector<F>& new_basis_2,
                                    const Vector<F>& new_basis_3)
    : origin(new_origin),
      basis_1(new_basis_1),
      basis_2(new_basis_2),
      basis_3(new_basis_3) {}

template <typename F>
Point<F> CoordinateFrame<F>::operator()(F coord_1, F coord_2, F coord_3) const
{
    return origin + basis_1*coord_1 + basis_2*coord_2 + basis_3*coord_3;
}

template <typename F>
CoordinateFrame<F>& CoordinateFrame<F>::rotateFromXToY(F angle)
{
    basis_1.rotateFromXToY(angle);
    basis_2.rotateFromXToY(angle);
    basis_3.rotateFromXToY(angle);
    return *this;
}

template <typename F>
CoordinateFrame<F>& CoordinateFrame<F>::rotateFromYToZ(F angle)
{
    basis_1.rotateFromYToZ(angle);
    basis_2.rotateFromYToZ(angle);
    basis_3.rotateFromYToZ(angle);
    return *this;
}

template <typename F>
CoordinateFrame<F>& CoordinateFrame<F>::rotateFromZToX(F angle)
{
    basis_1.rotateFromZToX(angle);
    basis_2.rotateFromZToX(angle);
    basis_3.rotateFromZToX(angle);
    return *this;
}

template <typename F>
CoordinateFrame<F>& CoordinateFrame<F>::makeAxesPerpendicular()
{
    basis_2 = basis_2.getProjectedOnNormalTo(basis_3, basis_1);
    basis_3 = basis_3.getProjectedOnNormalTo(basis_1, basis_2);
    return *this;
}

} // Geometry3D
