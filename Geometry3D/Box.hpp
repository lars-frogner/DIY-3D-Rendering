#pragma once
#include <vector>
#include "Point.hpp"
#include "Vector.hpp"
#include "Sphere.hpp"
#include "AxisAlignedBox.hpp"

namespace Geometry3D {

template <typename F>
class Box {

private:
    Vector<F> _width_vector, _height_vector, _depth_vector;
    F _width, _height, _depth;

public:
    Point<F> origin;

    Box<F>(const Point<F>&  new_origin,
           const Vector<F>& new_width_vector,
           const Vector<F>& new_height_vector,
           const Vector<F>& new_depth_vector);
    
    Box<F>(const Point<F>& new_origin,
           F new_width, F new_height, F new_depth);

    Box<F>(const Point<F>& new_origin,
           const Point<F>& width_point,
           const Point<F>& height_point,
           const Point<F>& depth_point);

    const Vector<F>& getWidthVector() const;
    const Vector<F>& getHeightVector() const;
    const Vector<F>& getDepthVector() const;
    F getWidth() const;
    F getHeight() const;
    F getDepth() const;
    Point<F> getCenter() const;
    std::vector< Point<F> > getCorners() const;

    Box<F>& setWidth(F new_width);
    Box<F>& setHeight(F new_height);
    Box<F>& setDepth(F new_depth);
    Box<F>& setDimensions(F new_width, F new_height, F new_depth);
    Box<F>& setCenter(const Point<F>& center);

    Box<F>& rotateFromXToY(F angle);
    Box<F>& rotateFromYToZ(F angle);
    Box<F>& rotateFromZToX(F angle);

    Box<F>& makeAxesPerpendicular();

    Sphere<F> getBoundingSphere() const;
    AxisAlignedBox<F> getAABB() const;
};

template <typename F>
Box<F>::Box(const Point<F>&  new_origin,
            const Vector<F>& new_width_vector,
            const Vector<F>& new_height_vector,
            const Vector<F>& new_depth_vector)
    : origin(new_origin),
      _width_vector(new_width_vector),
      _height_vector(new_height_vector),
      _depth_vector(new_depth_vector),
      _width(new_width_vector.getLength()),
      _height(new_height_vector.getLength()),
      _depth(new_depth_vector.getLength()) {}

template <typename F>
Box<F>::Box(const Point<F>& new_origin,
            F new_width, F new_height, F new_depth)
    : origin(new_origin),
      _width_vector(Vector<F>(new_width, 0, 0)),
      _height_vector(Vector<F>(0, new_height, 0)),
      _depth_vector(Vector<F>(0, 0, new_depth)),
      _width(new_width),
      _height(new_height),
      _depth(new_depth) {}

template <typename F>
Box<F>::Box(const Point<F>& new_origin,
            const Point<F>& width_point,
            const Point<F>& height_point,
            const Point<F>& depth_point)
    : origin(new_origin),
      _width_vector(width_point - new_origin),
      _height_vector(height_point - new_origin),
      _depth_vector(height_point - new_origin),
      _width((width_point - new_origin).getLength()),
      _height((height_point - new_origin).getLength()),
      _depth((height_point - new_origin).getLength()) {}

template <typename F>
const Vector<F>& Box<F>::getWidthVector() const
{
    return _width_vector;
}

template <typename F>
const Vector<F>& Box<F>::getHeightVector() const
{
    return _height_vector;
}

template <typename F>
const Vector<F>& Box<F>::getDepthVector() const
{
    return _depth_vector;
}

template <typename F>
F Box<F>::getWidth() const
{
    return _width;
}

template <typename F>
F Box<F>::getHeight() const
{
    return _height;
}

template <typename F>
F Box<F>::getDepth() const
{
    return _depth;
}

template <typename F>
Point<F> Box<F>::getCenter() const
{
    return origin + (_width_vector + _height_vector + _depth_vector)*0.5;
}

template <typename F>
std::vector< Point<F> > Box<F>::getCorners() const
{
    std::vector< Point<F> > corners;

    Point<F> corner_1 = origin + _width_vector;
    Point<F> corner_4 = origin + _depth_vector;
    Point<F> corner_5 = corner_4 + _width_vector;

    corners.push_back(origin);
    corners.push_back(corner_1);
    corners.push_back(corner_1 + _height_vector);
    corners.push_back(origin + _height_vector);
    corners.push_back(corner_4);
    corners.push_back(corner_5);
    corners.push_back(corner_5 + _height_vector);
    corners.push_back(corner_4 + _height_vector);

    return corners;
}

template <typename F>
Box<F>& Box<F>::setWidth(F new_width)
{
    _width_vector *= new_width/_width;
    _width = new_width;
    return *this;
}

template <typename F>
Box<F>& Box<F>::setHeight(F new_height)
{
    _height_vector *= new_height/_height;
    _height = new_height;
    return *this;
}

template <typename F>
Box<F>& Box<F>::setDepth(F new_depth)
{
    _depth_vector *= new_depth/_depth;
    _depth = new_depth;
    return *this;
}

template <typename F>
Box<F>& Box<F>::setDimensions(F new_width, F new_height, F new_depth)
{
    return setWidth(new_width).setHeight(new_height).setDepth(new_depth);
}

template <typename F>
Box<F>& Box<F>::setCenter(const Point<F>& center)
{
    origin = center - (_width_vector + _height_vector + _depth_vector)*0.5;
    return *this;
}

template <typename F>
Box<F>& Box<F>::rotateFromXToY(F angle)
{
    _width_vector.rotateFromXToY(angle);
    _height_vector.rotateFromXToY(angle);
    _depth_vector.rotateFromXToY(angle);
    return *this;
}

template <typename F>
Box<F>& Box<F>::rotateFromYToZ(F angle)
{
    _width_vector.rotateFromYToZ(angle);
    _height_vector.rotateFromYToZ(angle);
    _depth_vector.rotateFromYToZ(angle);
    return *this;
}

template <typename F>
Box<F>& Box<F>::rotateFromZToX(F angle)
{
    _width_vector.rotateFromZToX(angle);
    _height_vector.rotateFromZToX(angle);
    _depth_vector.rotateFromZToX(angle);
    return *this;
}

template <typename F>
Box<F>& Box<F>::makeAxesPerpendicular()
{
    _height_vector = _height_vector.getProjectedOnNormalTo(_depth_vector, _width_vector);
    _depth_vector = _depth_vector.getProjectedOnNormalTo(_width_vector, _height_vector);
    _height = _height_vector.getLength();
    _depth = _depth_vector.getLength();
    return *this;
}

template <typename F>
Sphere<F> Box<F>::getBoundingSphere() const
{
    const Vector<F>& radius_vector = (_width_vector + _height_vector + _depth_vector)*0.5;
    return Sphere<F>(origin + radius_vector,
                     radius_vector.getLength());
}

template <typename F>
AxisAlignedBox<F> Box<F>::getAABB() const
{
    const std::vector< Point<F> >& corners = getCorners();
    Point<F> min_point = corners[0];
    Point<F> max_point = min_point;

    for (size_t i = 1; i < 8; i++)
    {
        min_point.useSmallestCoordinates(corners[i]);
        max_point.useLargestCoordinates(corners[i]);
    }

    return AxisAlignedBox<F>(min_point, max_point);
}

} // Geometry3D
