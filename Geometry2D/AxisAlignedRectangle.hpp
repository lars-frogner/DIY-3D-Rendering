#pragma once
#include "assert.h"
#include "Point.hpp"
#include "Vector.hpp"

namespace Geometry2D {

template <typename F>
class AxisAlignedRectangle {

public:
    Point<F> lower_corner, upper_corner;

    AxisAlignedRectangle<F>(const Point<F>& new_lower_corner,
                            const Point<F>& new_upper_corner);

    Vector<F> getSpan() const;
    Point<F> getCenter() const;
    F getWidth() const;
    F getHeight() const;

    AxisAlignedRectangle<F>& setSpan(const Vector<F>& new_span);
    AxisAlignedRectangle<F>& setCenter(const Point<F>& center);
    AxisAlignedRectangle<F>& setWidth(F new_width);
    AxisAlignedRectangle<F>& setHeight(F new_height);
    AxisAlignedRectangle<F>& setDimensions(F new_width, F new_height);

    AxisAlignedRectangle<F>& translate(F dx, F dy);
    AxisAlignedRectangle<F>& translate(const Vector<F>& displacement);

    AxisAlignedRectangle<F>& validateOrientation();
};

template <typename F>
AxisAlignedRectangle<F>::AxisAlignedRectangle(const Point<F>& new_lower_corner,
                                              const Point<F>& new_upper_corner)
    : lower_corner(new_lower_corner),
      upper_corner(new_upper_corner) {}

template <typename F>
Vector<F> AxisAlignedRectangle<F>::getSpan() const
{
    return upper_corner - lower_corner;
}

template <typename F>
Point<F> AxisAlignedRectangle<F>::getCenter() const
{
    return lower_corner + getSpan()*0.5;
}

template <typename F>
F AxisAlignedRectangle<F>::getWidth() const
{
    return getSpan().x;
}

template <typename F>
F AxisAlignedRectangle<F>::getHeight() const
{
    return getSpan().y;
}

template <typename F>
AxisAlignedRectangle<F>& AxisAlignedRectangle<F>::setSpan(const Vector<F>& new_span)
{
    upper_corner = lower_corner + new_span;
    return *this;
}

template <typename F>
AxisAlignedRectangle<F>& AxisAlignedRectangle<F>::setCenter(const Point<F>& center)
{
    const Vector<F>& half_span = getSpan()*0.5;
    lower_corner = center - half_span;
    upper_corner = center + half_span;
    return *this;
}

template <typename F>
AxisAlignedRectangle<F>& AxisAlignedRectangle<F>::setWidth(F new_width)
{
    upper_corner.x += new_width - getSpan().x;
    return *this;
}

template <typename F>
AxisAlignedRectangle<F>& AxisAlignedRectangle<F>::setHeight(F new_height)
{
    upper_corner.y += new_height - getSpan().y;
    return *this;
}

template <typename F>
AxisAlignedRectangle<F>& AxisAlignedRectangle<F>::setDimensions(F new_width, F new_height)
{
    return setWidth(new_width).setHeight(new_height);
}

template <typename F>
AxisAlignedRectangle<F>& AxisAlignedRectangle<F>::translate(F dx, F dy)
{
    lower_corner.translate(dx, dy);
    upper_corner.translate(dx, dy);
    return *this;
}

template <typename F>
AxisAlignedRectangle<F>& AxisAlignedRectangle<F>::translate(const Vector<F>& displacement)
{
    lower_corner += displacement;
    upper_corner += displacement;
    return *this;
}

template <typename F>
AxisAlignedRectangle<F>& AxisAlignedRectangle<F>::validateOrientation()
{
    const Vector<F>& span = getSpan();
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

} // Geometry2D
