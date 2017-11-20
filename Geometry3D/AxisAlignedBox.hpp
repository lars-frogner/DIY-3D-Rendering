#pragma once
#include <vector>
#include <limits>
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
    F _INFINITY = std::numeric_limits<F>::infinity();

public:
    Point<F> lower_corner, upper_corner;

    AxisAlignedBox<F>();

    AxisAlignedBox<F>(const Point<F>& new_lower_corner,
                      const Point<F>& new_upper_corner);
    
    AxisAlignedBox<F>(const Point<F>&  new_lower_corner,
                      const Vector<F>& new_span);
    
    AxisAlignedBox<F>(const Point<F>&  new_lower_corner,
                      F width, F height, F depth);

    Vector<F> getSpan() const;
    Point<F> getCenter() const;
    F getWidth() const;
    F getHeight() const;
    F getDepth() const;

    std::vector< AxisAlignedBox<F> > getOctants() const;

    AxisAlignedBox<F>& setSpan(const Vector<F>& new_span);
    AxisAlignedBox<F>& setCenter(const Point<F>& center);
    AxisAlignedBox<F>& setWidth(F new_width);
    AxisAlignedBox<F>& setHeight(F new_height);
    AxisAlignedBox<F>& setDepth(F new_depth);
    AxisAlignedBox<F>& setDimensions(F new_width, F new_height, F new_depth);

    AxisAlignedBox<F>& translate(F dx, F dy, F dz);
    AxisAlignedBox<F>& translate(const Vector<F>& displacement);
    AxisAlignedBox<F> getTranslated(F dx, F dy, F dz) const;
    AxisAlignedBox<F> getTranslated(const Vector<F>& displacement) const;
    
    AxisAlignedBox<F>& merge(const AxisAlignedBox<F>& other);

    F evaluateRayIntersection(const Ray<F>& ray) const;
	bool intersects(const AxisAlignedBox<F>& other) const;
	bool encloses(const AxisAlignedBox<F>& other) const;

	bool containsInclusive(const Point<F>& point) const;
    bool containsUpperExclusive(const Point<F>& point) const;

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
Vector<F> AxisAlignedBox<F>::getSpan() const
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
std::vector< AxisAlignedBox<F> > AxisAlignedBox<F>::getOctants() const
{
    std::vector< AxisAlignedBox<F> > octants;

	octants.reserve(8);

	const Point<F>& center = getCenter();

	octants.push_back(*this);
	octants[0].upper_corner = center;

	octants.push_back(*this);
	octants[1].lower_corner.x = center.x;
	octants[1].upper_corner.y = center.y;
	octants[1].upper_corner.z = center.z;

	octants.push_back(*this);
	octants[2].lower_corner.y = center.y;
	octants[2].upper_corner.x = center.x;
	octants[2].upper_corner.z = center.z;

	octants.push_back(*this);
	octants[3].lower_corner.x = center.x;
	octants[3].lower_corner.y = center.y;
	octants[3].upper_corner.z = center.z;

	octants.push_back(*this);
	octants[4].lower_corner.z = center.z;
	octants[4].upper_corner.x = center.x;
	octants[4].upper_corner.y = center.y;

	octants.push_back(*this);
	octants[5].lower_corner.x = center.x;
	octants[5].lower_corner.z = center.z;
	octants[5].upper_corner.y = center.y;

	octants.push_back(*this);
	octants[6].lower_corner.y = center.y;
	octants[6].lower_corner.z = center.z;
	octants[6].upper_corner.x = center.x;

	octants.push_back(*this);
	octants[7].lower_corner = center;

    return octants;
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
AxisAlignedBox<F> AxisAlignedBox<F>::getTranslated(F dx, F dy, F dz) const
{
    return AxisAlignedBox<F>(*this).translate(dx, dy, dz);
}

template <typename F>
AxisAlignedBox<F> AxisAlignedBox<F>::getTranslated(const Vector<F>& displacement) const
{
    return AxisAlignedBox<F>(*this).translate(displacement);
}

template <typename F>
AxisAlignedBox<F>& AxisAlignedBox<F>::merge(const AxisAlignedBox<F>& other)
{
    lower_corner.useSmallestCoordinates(other.lower_corner);
    upper_corner.useLargestCoordinates(other.upper_corner);

    return *this;
}

template <typename F>
F AxisAlignedBox<F>::evaluateRayIntersection(const Ray<F>& ray) const
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
        return _INFINITY;

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
        return _INFINITY;

    if (min_dist_temp > min_dist)
        min_dist = min_dist_temp;

    if (max_dist_temp < max_dist)
        max_dist = max_dist_temp;

    if (max_dist >= 0 && min_dist < ray.max_distance)
        return min_dist;
    else
        return _INFINITY;
}

template <typename F>
bool AxisAlignedBox<F>::intersects(const AxisAlignedBox<F>& other) const
{
	const Point<F>& center_this = getCenter();
	const Point<F>& center_other = other.getCenter();
	const Vector<F>& span_this = getSpan();
	const Vector<F>& span_other = other.getSpan();

	return (2*abs(center_this.x - center_other.x) < (span_this.x + span_other.x)) &&
		   (2*abs(center_this.y - center_other.y) < (span_this.y + span_other.y)) &&
		   (2*abs(center_this.z - center_other.z) < (span_this.z + span_other.z));
}

template <typename F>
bool AxisAlignedBox<F>::encloses(const AxisAlignedBox<F>& other) const
{
	return (lower_corner.x < other.lower_corner.x) && (upper_corner.x > other.upper_corner.x) &&
		   (lower_corner.y < other.lower_corner.y) && (upper_corner.y > other.upper_corner.y) &&
		   (lower_corner.z < other.lower_corner.z) && (upper_corner.z > other.upper_corner.z);
}

template <typename F>
bool AxisAlignedBox<F>::containsInclusive(const Point<F>& point) const
{
	return (point.x >= lower_corner.x &&
			point.x <= upper_corner.x &&
			point.y >= lower_corner.y &&
			point.y <= upper_corner.y &&
			point.z >= lower_corner.z &&
			point.z <= upper_corner.z);
}

template <typename F>
bool AxisAlignedBox<F>::containsUpperExclusive(const Point<F>& point) const
{
    return (point.x >= lower_corner.x &&
            point.x < upper_corner.x &&
            point.y >= lower_corner.y &&
            point.y < upper_corner.y &&
            point.z >= lower_corner.z &&
            point.z < upper_corner.z);
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

template <typename F>
struct AABBContainer
{
    AxisAlignedBox<F> aabb;
    Point<F> centroid;
    size_t id;
};

} // Geometry3D
