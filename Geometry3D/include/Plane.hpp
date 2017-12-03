#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Geometry3D {

class Plane {

private:
    Vector _normal, _basis_1, _basis_2;
    bool _has_basis = false;

public:
    Point origin;

	Plane();

    Plane(const Point&  new_origin,
          const Vector& new_normal);

    Plane(const Point&  new_origin,
          const Vector& new_basis_1,
          const Vector& new_basis_2);

    Plane(const Point&  new_origin,
          const Point&  point_1,
          const Point&  point_2);

    Point operator()(imp_float distance_1, imp_float distance_2) const;

    const Vector& getNormalVector() const;
    const Vector& getBasisVector1() const;
    const Vector& getBasisVector2() const;

    bool hasBasis() const;

    Plane& setNormalVector(const Vector& new_normal);
    Plane& setBasisVectors(const Vector& new_basis_1,
                           const Vector& new_basis_2);

    Plane& normalizeNormalVector();
    Plane& normalizeBasisVectors();

    Plane& shiftInNormalDirection(imp_float distance);
    Plane& rotateFromXToY(imp_float angle);
    Plane& rotateFromYToZ(imp_float angle);
    Plane& rotateFromZToX(imp_float angle);

	bool hasOnPositiveSide(const Point& point) const;
};

} // Geometry3D
} // Impact
