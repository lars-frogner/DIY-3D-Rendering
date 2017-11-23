#include "AxisAlignedBox.hpp"
#include "Sphere.hpp"
#include <cassert>
#include <limits>

namespace Impact {
namespace Geometry3D {

AxisAlignedBox::AxisAlignedBox()
    : lower_corner(Point::min()),
      upper_corner(Point::max()) {}

AxisAlignedBox::AxisAlignedBox(const Point& new_lower_corner,
                               const Point& new_upper_corner)
    : lower_corner(new_lower_corner),
      upper_corner(new_upper_corner) {}

AxisAlignedBox::AxisAlignedBox(const Point&  new_lower_corner,
                               const Vector& new_span)
    : lower_corner(new_lower_corner),
      upper_corner(new_lower_corner + new_span) {}

AxisAlignedBox::AxisAlignedBox(const Point&  new_lower_corner,
                               imp_float width, imp_float height, imp_float depth)
    : lower_corner(new_lower_corner),
      upper_corner(Point(new_lower_corner).translate(width, height, depth)) {}

Vector AxisAlignedBox::getSpan() const
{
    return upper_corner - lower_corner;
}

Point AxisAlignedBox::getCenter() const
{
    return lower_corner + getSpan()*0.5;
}

imp_float AxisAlignedBox::getWidth() const
{
    return getSpan().x;
}

imp_float AxisAlignedBox::getHeight() const
{
    return getSpan().y;
}

imp_float AxisAlignedBox::getDepth() const
{
    return getSpan().z;
}

std::vector<AxisAlignedBox> AxisAlignedBox::getOctants() const
{
    std::vector<AxisAlignedBox> octants;

	octants.reserve(8);

	const Point& center = getCenter();

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

AxisAlignedBox& AxisAlignedBox::setSpan(const Vector& new_span)
{
    upper_corner = lower_corner + new_span;
    return *this;
}

AxisAlignedBox& AxisAlignedBox::setCenter(const Point& center)
{
    const Vector& half_span = getSpan()*0.5;
    lower_corner = center - half_span;
    upper_corner = center + half_span;
    return *this;
}

AxisAlignedBox& AxisAlignedBox::setWidth(imp_float new_width)
{
    upper_corner.x += new_width - getSpan().x;
    return *this;
}

AxisAlignedBox& AxisAlignedBox::setHeight(imp_float new_height)
{
    upper_corner.y += new_height - getSpan().y;
    return *this;
}

AxisAlignedBox& AxisAlignedBox::setDepth(imp_float new_depth)
{
    upper_corner.z += new_depth - getSpan().z;
    return *this;
}

AxisAlignedBox& AxisAlignedBox::setDimensions(imp_float new_width, imp_float new_height, imp_float new_depth)
{
    upper_corner = lower_corner + Vector(new_width, new_height, new_depth);
    return *this;
}

AxisAlignedBox& AxisAlignedBox::translate(imp_float dx, imp_float dy, imp_float dz)
{
    lower_corner.translate(dx, dy, dz);
    upper_corner.translate(dx, dy, dz);
    return *this;
}

AxisAlignedBox& AxisAlignedBox::translate(const Vector& displacement)
{
    lower_corner += displacement;
    upper_corner += displacement;
    return *this;
}

AxisAlignedBox AxisAlignedBox::getTranslated(imp_float dx, imp_float dy, imp_float dz) const
{
    return AxisAlignedBox(*this).translate(dx, dy, dz);
}

AxisAlignedBox AxisAlignedBox::getTranslated(const Vector& displacement) const
{
    return AxisAlignedBox(*this).translate(displacement);
}

AxisAlignedBox& AxisAlignedBox::merge(const AxisAlignedBox& other)
{
    lower_corner.useSmallestCoordinates(other.lower_corner);
    upper_corner.useLargestCoordinates(other.upper_corner);

    return *this;
}

imp_float AxisAlignedBox::evaluateRayIntersection(const Ray& ray) const
{
    imp_float min_dist, max_dist, min_dist_temp, max_dist_temp;
    const Vector& inverse_direction = ray.inverse_direction;

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
        return IMP_FLOAT_INF;

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
        return IMP_FLOAT_INF;

    if (min_dist_temp > min_dist)
        min_dist = min_dist_temp;

    if (max_dist_temp < max_dist)
        max_dist = max_dist_temp;

    if (max_dist >= 0 && min_dist < ray.max_distance)
        return min_dist;
    else
        return IMP_FLOAT_INF;
}

bool AxisAlignedBox::intersects(const AxisAlignedBox& other) const
{
	const Point& center_this = getCenter();
	const Point& center_other = other.getCenter();
	const Vector& span_this = getSpan();
	const Vector& span_other = other.getSpan();

	return (2*abs(center_this.x - center_other.x) < (span_this.x + span_other.x)) &&
		   (2*abs(center_this.y - center_other.y) < (span_this.y + span_other.y)) &&
		   (2*abs(center_this.z - center_other.z) < (span_this.z + span_other.z));
}

bool AxisAlignedBox::encloses(const AxisAlignedBox& other) const
{
	return (lower_corner.x < other.lower_corner.x) && (upper_corner.x > other.upper_corner.x) &&
		   (lower_corner.y < other.lower_corner.y) && (upper_corner.y > other.upper_corner.y) &&
		   (lower_corner.z < other.lower_corner.z) && (upper_corner.z > other.upper_corner.z);
}

bool AxisAlignedBox::containsInclusive(const Point& point) const
{
	return (point.x >= lower_corner.x &&
			point.x <= upper_corner.x &&
			point.y >= lower_corner.y &&
			point.y <= upper_corner.y &&
			point.z >= lower_corner.z &&
			point.z <= upper_corner.z);
}

bool AxisAlignedBox::containsUpperExclusive(const Point& point) const
{
    return (point.x >= lower_corner.x &&
            point.x < upper_corner.x &&
            point.y >= lower_corner.y &&
            point.y < upper_corner.y &&
            point.z >= lower_corner.z &&
            point.z < upper_corner.z);
}

Sphere AxisAlignedBox::getBoundingSphere() const
{
    const Vector& radius_vector = getSpan()*0.5;
    return Sphere(lower_corner + radius_vector,
                  radius_vector.getLength());
}

AxisAlignedBox& AxisAlignedBox::validateOrientation()
{
    const Vector& span = getSpan();

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
} // Impact
