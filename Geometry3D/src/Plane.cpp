#pragma once
#include "Plane.hpp"
#include <cassert>

namespace Impact {
namespace Geometry3D {

Plane::Plane() {}

Plane::Plane(const Point&  new_origin,
             const Vector& new_normal)
    : origin(new_origin), _normal(new_normal) {}

Plane::Plane(const Point&  new_origin,
             const Vector& new_basis_1,
             const Vector& new_basis_2)
    : origin(new_origin),
      _normal(new_basis_1.getUnitNormalWith(new_basis_2)),
      _basis_1(new_basis_1),
      _basis_2(new_basis_2),
      _has_basis(true) {}

Plane::Plane(const Point&  new_origin,
             const Point&  point_1,
             const Point&  point_2)
    : origin(new_origin),
      _basis_1(point_1 - new_origin),
      _basis_2(point_2 - new_origin),
      _normal((point_1 - new_origin).getUnitNormalWith(point_2 - new_origin)),
      _has_basis(true) {}

Point Plane::operator()(imp_float distance_1, imp_float distance_2) const
{
    assert(_has_basis);
    return origin + _basis_1*distance_1 + _basis_2*distance_2;
}

const Vector& Plane::getNormalVector() const
{
    return _normal;
}

const Vector& Plane::getBasisVector1() const
{
    assert(_has_basis);
    return _basis_1;
}

const Vector& Plane::getBasisVector2() const
{
    assert(_has_basis);
    return _basis_2;
}

bool Plane::hasBasis() const
{
    return _has_basis;
}

Plane& Plane::setNormalVector(const Vector& new_normal)
{
    _normal = new_normal;
    _basis_1 = Vector::zero();
    _basis_2 = Vector::zero();
    _has_basis = false;
    return *this;
}

Plane& Plane::setBasisVectors(const Vector& new_basis_1,
                              const Vector& new_basis_2)
{
    _basis_1 = new_basis_1;
    _basis_2 = new_basis_2;
    _normal = new_basis_1.getUnitNormalWith(new_basis_2);
    _has_basis = true;
    return *this;
}

Plane& Plane::normalizeNormalVector()
{
    _normal.normalize();
    return *this;
}

Plane& Plane::normalizeBasisVectors()
{
    assert(_has_basis);
    _basis_1.normalize();
    _basis_2.normalize();
    return *this;
}

Plane& Plane::shiftInNormalDirection(imp_float distance)
{
    origin += _normal*distance;
    return *this;
}

Plane& Plane::rotateFromXToY(imp_float angle)
{
    _normal.rotateFromXToY(angle);
    if (_has_basis)
    {
        _basis_1.rotateFromXToY(angle);
        _basis_2.rotateFromXToY(angle);
    }
    return *this;
}

Plane& Plane::rotateFromYToZ(imp_float angle)
{
    _normal.rotateFromYToZ(angle);
    if (_has_basis)
    {
        _basis_1.rotateFromYToZ(angle);
        _basis_2.rotateFromYToZ(angle);
    }
    return *this;
}

Plane& Plane::rotateFromZToX(imp_float angle)
{
    _normal.rotateFromZToX(angle);
    if (_has_basis)
    {
        _basis_1.rotateFromZToX(angle);
        _basis_2.rotateFromZToX(angle);
    }
    return *this;
}

bool Plane::hasOnPositiveSide(const Point& point) const
{
	return _normal.dot((point - origin).getNormalized()) > 1e-6;
}

} // Geometry3D
} // Impact
