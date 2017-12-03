#include "AxisAlignedRectangle.hpp"
#include <cassert>

namespace Impact {
namespace Geometry2D {

AxisAlignedRectangle::AxisAlignedRectangle()
    : lower_corner(Point::min()),
      upper_corner(Point::max()) {}

AxisAlignedRectangle::AxisAlignedRectangle(const Point& new_lower_corner,
                                           const Point& new_upper_corner)
    : lower_corner(new_lower_corner),
      upper_corner(new_upper_corner) {}

Vector AxisAlignedRectangle::getSpan() const
{
    return upper_corner - lower_corner;
}

Point AxisAlignedRectangle::getCenter() const
{
    return lower_corner + getSpan()*0.5;
}

imp_float AxisAlignedRectangle::getWidth() const
{
    return getSpan().x;
}

imp_float AxisAlignedRectangle::getHeight() const
{
    return getSpan().y;
}

std::vector<AxisAlignedRectangle> AxisAlignedRectangle::getQuadrants() const
{
    std::vector<AxisAlignedRectangle> quadrants;

	quadrants.reserve(4);

	const Point& center = getCenter();

	quadrants.push_back(*this);
	quadrants[0].upper_corner = center;

	quadrants.push_back(*this);
	quadrants[1].lower_corner.x = center.x;
	quadrants[1].upper_corner.y = center.y;

	quadrants.push_back(*this);
	quadrants[2].lower_corner.y = center.y;
	quadrants[2].upper_corner.x = center.x;

	quadrants.push_back(*this);
	quadrants[3].lower_corner = center;

    return quadrants;
}

AxisAlignedRectangle& AxisAlignedRectangle::setSpan(const Vector& new_span)
{
    upper_corner = lower_corner + new_span;
    return *this;
}

AxisAlignedRectangle& AxisAlignedRectangle::setCenter(const Point& center)
{
    const Vector& half_span = getSpan()*0.5;
    lower_corner = center - half_span;
    upper_corner = center + half_span;
    return *this;
}

AxisAlignedRectangle& AxisAlignedRectangle::setWidth(imp_float new_width)
{
    upper_corner.x += new_width - getSpan().x;
    return *this;
}

AxisAlignedRectangle& AxisAlignedRectangle::setHeight(imp_float new_height)
{
    upper_corner.y += new_height - getSpan().y;
    return *this;
}

AxisAlignedRectangle& AxisAlignedRectangle::setDimensions(imp_float new_width, imp_float new_height)
{
    return setWidth(new_width).setHeight(new_height);
}

bool AxisAlignedRectangle::containsInclusive(const Point& point) const
{
    return (point.x >= lower_corner.x &&
            point.x <= upper_corner.x &&
            point.y >= lower_corner.y &&
            point.y <= upper_corner.y);
}

bool AxisAlignedRectangle::containsUpperExclusive(const Point& point) const
{
    return (point.x >= lower_corner.x &&
            point.x < upper_corner.x &&
            point.y >= lower_corner.y &&
            point.y < upper_corner.y);
}

AxisAlignedRectangle& AxisAlignedRectangle::translate(imp_float dx, imp_float dy)
{
    lower_corner.translate(dx, dy);
    upper_corner.translate(dx, dy);
    return *this;
}

AxisAlignedRectangle& AxisAlignedRectangle::translate(const Vector& displacement)
{
    lower_corner += displacement;
    upper_corner += displacement;
    return *this;
}

AxisAlignedRectangle AxisAlignedRectangle::getTranslated(imp_float dx, imp_float dy) const
{
    return AxisAlignedRectangle(*this).translate(dx, dy);
}

AxisAlignedRectangle AxisAlignedRectangle::getTranslated(const Vector& displacement) const
{
    return AxisAlignedRectangle(*this).translate(displacement);
}

AxisAlignedRectangle& AxisAlignedRectangle::merge(const AxisAlignedRectangle& other)
{
    lower_corner.useSmallestCoordinates(other.lower_corner);
    upper_corner.useLargestCoordinates(other.upper_corner);

    return *this;
}

AxisAlignedRectangle& AxisAlignedRectangle::validateOrientation()
{
    const Vector& span = getSpan();
    bool width_is_negative = span.x < 0;
    bool height_is_negative = span.y < 0;

    if (width_is_negative && height_is_negative)
    {
        lower_corner.swap(upper_corner);
    }
    else
    {
        assert(!width_is_negative);
        assert(!height_is_negative);
    }
    return *this;
}

AABRContainer::AABRContainer(const AxisAlignedRectangle& new_aabr,
							 const Point& new_centroid,
							 imp_uint new_id)
	: aabr(new_aabr),
	  centroid(new_centroid),
	  id(new_id) {}

} // Geometry2D
} // Impact
