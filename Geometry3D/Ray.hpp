#pragma once
#include "Point.hpp"
#include "Vector.hpp"

namespace Geometry3D {

template <typename F>
class Ray {

public:
    Point<F> origin;
    Vector<F> direction;
    Vector<F> inverse_direction;

    Ray<F>(const Point<F>&  new_origin,
           const Vector<F>& new_direction);

    Point<F> operator()(F distance) const;

    Ray<F>& alignWith(const Vector<F>& other_direction);
    Ray<F>& pointAt(const Point<F>& point);
};

template <typename F>
Ray<F>::Ray(const Point<F>&  new_origin,
            const Vector<F>& new_direction)
    : origin(new_origin),
      direction(new_direction),
      inverse_direction(1.0f/new_direction) {}

template <typename F>
Point<F> Ray<F>::operator()(F distance) const
{
    return origin + direction*distance;
}

template <typename F>
Ray<F>& Ray<F>::alignWith(const Vector<F>& other_direction)
{
    direction = other_direction.getNormalized();
    return *this;
}

template <typename F>
Ray<F>& Ray<F>::pointAt(const Point<F>& point)
{
    return alignWith(point - origin);
}

} // Geometry3D
