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

public:
    Point<F> lower_corner, upper_corner;

    AxisAlignedBox<F>();

    AxisAlignedBox<F>(const Point<F>& new_lower_corner,
                      const Point<F>& new_upper_corner);
    
    AxisAlignedBox<F>(const Point<F>&  new_lower_corner,
                      const Vector<F>& new_span);
    
    AxisAlignedBox<F>(const Point<F>&  new_lower_corner,
                      F width, F height, F depth);

    const Vector<F>& getSpan() const;
    Point<F> getCenter() const;
    F getWidth() const;
    F getHeight() const;
    F getDepth() const;

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
AxisAlignedBox<F>::AxisAlignedBox()
    : lower_corner(Point<F>::min()),
      upper_corner(Point<F>::max()) {}

template <typename F>
AxisAlignedBox<F>::AxisAlignedBox(const Point<F>& new_lower_corner,
                                  const Point<F>& new_upper_corner)
    : lower_corner(new_lower_corner),
      upper_corner(new_upper_corner) {}

template <typename F>
AxisAlignedBox<F>::AxisAlignedBox(const Point<F>&  new_lower_corner,
                                  const Vector<F>& new_span)
    : lower_corner(new_lower_corner),
      upper_corner(new_lower_corner + new_span) {}

template <typename F>
AxisAlignedBox<F>::AxisAlignedBox(const Point<F>&  new_lower_corner,
                                  F width, F height, F depth)
    : lower_corner(new_lower_corner),
      upper_corner(Vector<F>(new_lower_corner).translate(width, height, depth)) {}

template <typename F>
const Vector<F>& AxisAlignedBox<F>::getSpan() const
{
    return upper_corner - lower_corner;
}

template <typename F>
Point<F> AxisAlignedBox<F>::getCenter() const
{
    return lower_corner + getSpan()*0.5;
}

template <typename F>
F AxisAlignedBox<F>::getWidth() const
{
    return getSpan().x;
}

template <typename F>
F AxisAlignedBox<F>::getHeight() const
{
    return getSpan().y;
}

template <typename F>
F AxisAlignedBox<F>::getDepth() const
{
    return getSpan().z;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setSpan(const Vector<F>& new_span)
{
    upper_corner = lower_corner + new_span;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setCenter(const Point<F>& center)
{
    const Vector<F>& half_span = getSpan()*0.5;
    lower_corner = center - half_span;
    upper_corner = center + half_span;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setWidth(F new_width)
{
    upper_corner.x += new_width - getSpan().x;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setHeight(F new_height)
{
    upper_corner.y += new_height - getSpan().y;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setDepth(F new_depth)
{
    upper_corner.z += new_depth - getSpan().z;
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::setDimensions(F new_width, F new_height, F new_depth)
{
    upper_corner = lower_corner + Vector<F>(new_width, new_height, new_depth);
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::translate(F dx, F dy, F dz)
{
    lower_corner.translate(dx, dy, dz);
    upper_corner.translate(dx, dy, dz);
    return *this;
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::translate(const Vector<F>& displacement)
{
    lower_corner += displacement;
    upper_corner += displacement;
    return *this;
}

template <typename F>
bool AxisAlignedBox<F>::evaluateRayIntersection(const Ray<F>& ray, F upper_distance_limit) const
{
    F min_dist, max_dist, min_dist_temp, max_dist_temp;
    const Vector<F>& inverse_direction = ray.inverse_direction;

    if (inverse_direction.x >= 0)
    {
        min_dist = (lower_corner.x - ray.origin.x)*inverse_direction.x;
        max_dist = (upper_corner.x - ray.origin.x)*inverse_direction.x;
    }
    else
    {
        min_dist = (upper_corner.x - ray.origin.x)*inverse_direction.x;
        max_dist = (lower_corner.x - ray.origin.x)*inverse_direction.x;
    }

    if (inverse_direction.y >= 0)
    {
        min_dist_temp = (lower_corner.y - ray.origin.y)*inverse_direction.y;
        max_dist_temp = (upper_corner.y - ray.origin.y)*inverse_direction.y;
    }
    else
    {
        min_dist_temp = (upper_corner.y - ray.origin.y)*inverse_direction.y;
        max_dist_temp = (lower_corner.y - ray.origin.y)*inverse_direction.y;
    }

    if (min_dist > max_dist_temp || min_dist_temp > max_dist)
        return false;

    if (min_dist_temp > min_dist)
        min_dist = min_dist_temp;

    if (max_dist_temp < max_dist)
        max_dist = max_dist_temp;

    if (inverse_direction.z >= 0)
    {
        min_dist_temp = (lower_corner.z - ray.origin.z)*inverse_direction.z;
        max_dist_temp = (upper_corner.z - ray.origin.z)*inverse_direction.z;
    }
    else
    {
        min_dist_temp = (upper_corner.z - ray.origin.z)*inverse_direction.z;
        max_dist_temp = (lower_corner.z - ray.origin.z)*inverse_direction.z;
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
    const Vector<F>& radius_vector = getSpan()*0.5;
    return Sphere<F>(lower_corner + radius_vector,
                     radius_vector.getLength());
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::validateOrientation()
{
    const Vector<f>& span = getSpan();

    bool width_is_negative = span.x < 0;
    bool height_is_negative = span.y < 0;
    bool depth_is_negative = span.z < 0;

    if (width_is_negative && height_is_negative && depth_is_negative)
    {
        lower_corner.swap(upper_corner);
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
