#pragma once
#include "assert.h"
#include "Point.hpp"
#include "Vector.hpp"
#include "Ray.hpp"

namespace Geometry3D {

// Forward declaration of Sphere class
template <typename F>
class Sphere;

template <typename F>
class AxisAlignedBox {

private:
    Point<F> _lower_corner, _upper_corner;
    Vector<F> _span;

public:
    AxisAlignedBox<F>(const Point<F>& new_lower_corner,
                      const Point<F>& new_upper_corner);
    
    AxisAlignedBox<F>(const Point<F>&  new_lower_corner,
                      const Vector<F>& new_span);
    
    AxisAlignedBox<F>(const Point<F>&  new_lower_corner,
                      F width, F height, F depth);

    const Point<F>& getLowerCorner() const;
    const Point<F>& getUpperCorner() const;
    const Vector<F>& getSpan() const;
    Point<F> getCenter() const;
    F getWidth() const;
    F getHeight() const;
    F getDepth() const;

    AxisAlignedBox<F>& setLowerCorner(const Point<F>& new_lower_corner);
    AxisAlignedBox<F>& setUpperCorner(const Point<F>& new_upper_corner);
    AxisAlignedBox<F>& setLowerAndUpperCorner(const Point<F>& new_lower_corner, const Point<F>& new_upper_corner);
    AxisAlignedBox<F>& setSpan(const Vector<F>& new_span);
    AxisAlignedBox<F>& setCenter(const Point<F>& center);
    AxisAlignedBox<F>& setWidth(F new_width);
    AxisAlignedBox<F>& setHeight(F new_height);
    AxisAlignedBox<F>& setDepth(F new_depth);
    AxisAlignedBox<F>& setDimensions(F new_width, F new_height, F new_depth);

    AxisAlignedBox<F>& translate(F dx, F dy, F dz);
    AxisAlignedBox<F>& translate(const Vector<F>& displacement);

    bool evaluateRayIntersection(const Ray<F>& ray, F upper_distance_limit) const;

    Sphere<F> getBoundingSphere() const;

    AxisAlignedBox<F>& validateOrientation();
};

template <typename F>
AxisAlignedBox<F>::AxisAlignedBox(const Point<F>& new_lower_corner,
                                  const Point<F>& new_upper_corner)
    : _lower_corner(new_lower_corner),
      _upper_corner(new_upper_corner),
      _span(new_upper_corner - new_lower_corner) {}

template <typename F>
AxisAlignedBox<F>::AxisAlignedBox(const Point<F>&  new_lower_corner,
                                  const Vector<F>& new_span)
    : _lower_corner(new_lower_corner),
      _upper_corner(new_lower_corner + new_span),
      _span(new_span) {}

template <typename F>
AxisAlignedBox<F>::AxisAlignedBox(const Point<F>&  new_lower_corner,
                                  F width, F height, F depth)
    : _span(Vector<F>(width, height, depth)),
      _lower_corner(new_lower_corner),
      _upper_corner(Vector<F>(new_lower_corner).translate(width, height, depth)) {}

template <typename F>
const Point<F>& AxisAlignedBox<F>::getLowerCorner() const
{
    return _lower_corner;
}

template <typename F>
const Point<F>& AxisAlignedBox<F>::getUpperCorner() const
{
    return _upper_corner;
}

template <typename F>
const Vector<F>& AxisAlignedBox<F>::getSpan() const
{
    return _span;
}

template <typename F>
Point<F> AxisAlignedBox<F>::getCenter() const
{
    return _lower_corner + _span*0.5;
}

template <typename F>
F AxisAlignedBox<F>::getWidth() const
{
    return _span.x;
}

template <typename F>
F AxisAlignedBox<F>::getHeight() const
{
    return _span.y;
}

template <typename F>
F AxisAlignedBox<F>::getDepth() const
{
    return _span.z;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setLowerCorner(const Point<F>& new_lower_corner)
{
    _lower_corner = new_lower_corner;
    _span = _upper_corner - new_lower_corner;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setUpperCorner(const Point<F>& new_upper_corner)
{
    _upper_corner = new_upper_corner;
    _span = new_upper_corner - _lower_corner;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setLowerAndUpperCorner(const Point<F>& new_lower_corner,
                                                             const Point<F>& new_upper_corner)
{
    _lower_corner = new_lower_corner;
    _upper_corner = new_upper_corner;
    _span = new_upper_corner - new_lower_corner;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setSpan(const Vector<F>& new_span)
{
    _span = new_span;
    _upper_corner = _lower_corner + new_span;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setCenter(const Point<F>& center)
{
    const Vector<F>& half_span = _span*0.5;
    _lower_corner = center - half_span;
    _upper_corner = center + half_span;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setWidth(F new_width)
{
    _upper_corner.x += new_width - _span.x;
    _span.x = new_width;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setHeight(F new_height)
{
    _upper_corner.y += new_height - _span.y;
    _span.y = new_height;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setDepth(F new_depth)
{
    _upper_corner.z += new_depth - _span.z;
    _span.z = new_depth;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setDimensions(F new_width, F new_height, F new_depth)
{
    _span.setComponents(new_width, new_height, new_depth);
    _upper_corner = _lower_corner + _span;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::translate(F dx, F dy, F dz)
{
    _lower_corner.translate(dx, dy, dz);
    _upper_corner.translate(dx, dy, dz);
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::translate(const Vector<F>& displacement)
{
    _lower_corner += displacement;
    _upper_corner += displacement;
    return *this;
}

template <typename F>
bool AxisAlignedBox<F>::evaluateRayIntersection(const Ray<F>& ray, F upper_distance_limit) const
{
    F min_dist, max_dist, min_dist_temp, max_dist_temp;
    const Vector<F>& inverse_direction = 1.0f/ray.direction;

    if (inverse_direction.x >= 0)
    {
        min_dist = (_lower_corner.x - ray.origin.x)*inverse_direction.x;
        max_dist = (_upper_corner.x - ray.origin.x)*inverse_direction.x;
    }
    else
    {
        min_dist = (_upper_corner.x - ray.origin.x)*inverse_direction.x;
        max_dist = (_lower_corner.x - ray.origin.x)*inverse_direction.x;
    }

    if (inverse_direction.y >= 0)
    {
        min_dist_temp = (_lower_corner.y - ray.origin.y)*inverse_direction.y;
        max_dist_temp = (_upper_corner.y - ray.origin.y)*inverse_direction.y;
    }
    else
    {
        min_dist_temp = (_upper_corner.y - ray.origin.y)*inverse_direction.y;
        max_dist_temp = (_lower_corner.y - ray.origin.y)*inverse_direction.y;
    }

    if (min_dist > max_dist_temp || min_dist_temp > max_dist)
        return false;

    if (min_dist_temp > min_dist)
        min_dist = min_dist_temp;

    if (max_dist_temp < max_dist)
        max_dist = max_dist_temp;

    if (inverse_direction.z >= 0)
    {
        min_dist_temp = (_lower_corner.z - ray.origin.z)*inverse_direction.z;
        max_dist_temp = (_upper_corner.z - ray.origin.z)*inverse_direction.z;
    }
    else
    {
        min_dist_temp = (_upper_corner.z - ray.origin.z)*inverse_direction.z;
        max_dist_temp = (_lower_corner.z - ray.origin.z)*inverse_direction.z;
    }

    if (min_dist > max_dist_temp || min_dist_temp > max_dist)
        return false;

    if (min_dist_temp > min_dist)
        min_dist = min_dist_temp;

    if (max_dist_temp < max_dist)
        max_dist = max_dist_temp;

    return (max_dist >= 0 && min_dist < upper_distance_limit);
}

template <typename F>
Sphere<F> AxisAlignedBox<F>::getBoundingSphere() const
{
    const Vector<F>& radius_vector = _span*0.5;
    return Sphere<F>(_lower_corner + radius_vector,
                     radius_vector.getLength());
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::validateOrientation()
{
    bool width_is_negative = _span.x < 0;
    bool height_is_negative = _span.y < 0;
    bool depth_is_negative = _span.z < 0;

    if (width_is_negative && height_is_negative && depth_is_negative)
    {
        _lower_corner.swap(_upper_corner);
        _span = -_span;
    }
    else
    {
        assert(!width_is_negative);
        assert(!height_is_negative);
        assert(!depth_is_negative);
    }
    return *this;
}

} // Geometry3D
