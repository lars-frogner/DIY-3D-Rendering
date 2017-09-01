#pragma once
#include <assert.h>
#include "Point.hpp"
#include "Vector.hpp"

namespace Geometry3D {

template <typename F>
class Plane {

private:
    Vector<F> _normal, _basis_1, _basis_2;
    bool _has_basis = false;

public:
    Point<F> origin;

    Plane<F>(const Point<F>&  new_origin,
             const Vector<F>& new_normal);

    Plane<F>(const Point<F>&  new_origin,
             const Vector<F>& new_basis_1,
             const Vector<F>& new_basis_2);

    Plane<F>(const Point<F>&  new_origin,
             const Point<F>&  point_1,
             const Point<F>&  point_2);

    Point<F> operator()(F distance_1, F distance_2) const;

    const Vector<F>& getNormalVector() const;
    const Vector<F>& getBasisVector1() const;
    const Vector<F>& getBasisVector2() const;

    bool hasBasis() const;

    Plane<F>& setNormalVector(const Vector<F>& new_normal);
    Plane<F>& setBasisVectors(const Vector<F>& new_basis_1,
                              const Vector<F>& new_basis_2);

    Plane<F>& normalizeNormalVector();
    Plane<F>& normalizeBasisVectors();

    Plane<F>& shiftInNormalDirection(F distance);
    Plane<F>& rotateFromXToY(F angle);
    Plane<F>& rotateFromYToZ(F angle);
    Plane<F>& rotateFromZToX(F angle);
};

template <typename F>
Plane<F>::Plane(const Point<F>&  new_origin,
                const Vector<F>& new_normal)
    : origin(new_origin), _normal(new_normal) {}

template <typename F>
Plane<F>::Plane(const Point<F>&  new_origin,
                const Vector<F>& new_basis_1,
                const Vector<F>& new_basis_2)
    : origin(new_origin),
      _normal(new_basis_1.getUnitNormalWith(new_basis_2)),
      _basis_1(new_basis_1),
      _basis_2(new_basis_2),
      _has_basis(true) {}

template <typename F>
Plane<F>::Plane(const Point<F>&  new_origin,
                const Point<F>&  point_1,
                const Point<F>&  point_2)
    : origin(new_origin),
      _basis_1(point_1 - new_origin),
      _basis_2(point_2 - new_origin),
      _normal((point_1 - new_origin).getUnitNormalWith(point_2 - new_origin)),
      _has_basis(true) {}

template <typename F>
Point<F> Plane<F>::operator()(F distance_1, F distance_2) const
{
    assert(_has_basis);
    return origin + _basis_1*distance_1 + _basis_2*distance_2;
}

template <typename F>
const Vector<F>& Plane<F>::getNormalVector() const
{
    return _normal;
}

template <typename F>
const Vector<F>& Plane<F>::getBasisVector1() const
{
    assert(_has_basis);
    return _basis_1;
}

template <typename F>
const Vector<F>& Plane<F>::getBasisVector2() const
{
    assert(_has_basis);
    return _basis_2;
}

template <typename F>
bool Plane<F>::hasBasis() const
{
    return _has_basis;
}

template <typename F>
Plane<F>& Plane<F>::setNormalVector(const Vector<F>& new_normal)
{
    _normal = new_normal;
    _basis_1 = Vector<F>::zero();
    _basis_2 = Vector<F>::zero();
    _has_basis = false;
    return *this;
}

template <typename F>
Plane<F>& Plane<F>::setBasisVectors(const Vector<F>& new_basis_1,
                                    const Vector<F>& new_basis_2)
{
    _basis_1 = new_basis_1;
    _basis_2 = new_basis_2;
    _normal = new_basis_1.getUnitNormalWith(new_basis_2);
    _has_basis = true;
    return *this;
}

template <typename F>
Plane<F>& Plane<F>::normalizeNormalVector()
{
    _normal.normalize();
    return *this;
}

template <typename F>
Plane<F>& Plane<F>::normalizeBasisVectors()
{
    assert(_has_basis);
    _basis_1.normalize();
    _basis_2.normalize();
    return *this;
}

template <typename F>
Plane<F>& Plane<F>::shiftInNormalDirection(F distance)
{
    origin += _normal*distance;
    return *this;
}

template <typename F>
Plane<F>& Plane<F>::rotateFromXToY(F angle)
{
    _normal.rotateFromXToY(angle);
    if (_has_basis)
    {
        _basis_1.rotateFromXToY(angle);
        _basis_2.rotateFromXToY(angle);
    }
    return *this;
}

template <typename F>
Plane<F>& Plane<F>::rotateFromYToZ(F angle)
{
    _normal.rotateFromYToZ(angle);
    if (_has_basis)
    {
        _basis_1.rotateFromYToZ(angle);
        _basis_2.rotateFromYToZ(angle);
    }
    return *this;
}

template <typename F>
Plane<F>& Plane<F>::rotateFromZToX(F angle)
{
    _normal.rotateFromZToX(angle);
    if (_has_basis)
    {
        _basis_1.rotateFromZToX(angle);
        _basis_2.rotateFromZToX(angle);
    }
    return *this;
}

} // Geometry3D
