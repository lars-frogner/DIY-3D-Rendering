#include "Box.hpp"

namespace Impact {
namespace Geometry3D {

Box::Box(const Point&  new_origin,
         const Vector& new_width_vector,
         const Vector& new_height_vector,
         const Vector& new_depth_vector)
    : origin(new_origin),
      _width_vector(new_width_vector),
      _height_vector(new_height_vector),
      _depth_vector(new_depth_vector),
      _width(new_width_vector.getLength()),
      _height(new_height_vector.getLength()),
      _depth(new_depth_vector.getLength()) {}

Box::Box(const Point& new_origin,
         imp_float new_width, imp_float new_height, imp_float new_depth)
    : origin(new_origin),
      _width_vector(Vector(new_width, 0, 0)),
      _height_vector(Vector(0, new_height, 0)),
      _depth_vector(Vector(0, 0, new_depth)),
      _width(new_width),
      _height(new_height),
      _depth(new_depth) {}

Box::Box(const Point& new_origin,
         const Point& width_point,
         const Point& height_point,
         const Point& depth_point)
    : origin(new_origin),
      _width_vector(width_point - new_origin),
      _height_vector(height_point - new_origin),
      _depth_vector(height_point - new_origin),
      _width((width_point - new_origin).getLength()),
      _height((height_point - new_origin).getLength()),
      _depth((height_point - new_origin).getLength()) {}

const Vector& Box::getWidthVector() const
{
    return _width_vector;
}

const Vector& Box::getHeightVector() const
{
    return _height_vector;
}

const Vector& Box::getDepthVector() const
{
    return _depth_vector;
}

imp_float Box::getWidth() const
{
    return _width;
}

imp_float Box::getHeight() const
{
    return _height;
}

imp_float Box::getDepth() const
{
    return _depth;
}

Point Box::getCenter() const
{
    return origin + (_width_vector + _height_vector + _depth_vector)*0.5;
}

std::vector<Point> Box::getCorners() const
{
    std::vector<Point> corners;

    Point corner_1 = origin + _width_vector;
    Point corner_4 = origin + _depth_vector;
    Point corner_5 = corner_4 + _width_vector;

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

Box& Box::setWidth(imp_float new_width)
{
    _width_vector *= new_width/_width;
    _width = new_width;
    return *this;
}

Box& Box::setHeight(imp_float new_height)
{
    _height_vector *= new_height/_height;
    _height = new_height;
    return *this;
}

Box& Box::setDepth(imp_float new_depth)
{
    _depth_vector *= new_depth/_depth;
    _depth = new_depth;
    return *this;
}

Box& Box::setDimensions(imp_float new_width, imp_float new_height, imp_float new_depth)
{
    return setWidth(new_width).setHeight(new_height).setDepth(new_depth);
}

Box& Box::setCenter(const Point& center)
{
    origin = center - (_width_vector + _height_vector + _depth_vector)*0.5;
    return *this;
}

Box& Box::rotateFromXToY(imp_float angle)
{
    _width_vector.rotateFromXToY(angle);
    _height_vector.rotateFromXToY(angle);
    _depth_vector.rotateFromXToY(angle);
    return *this;
}

Box& Box::rotateFromYToZ(imp_float angle)
{
    _width_vector.rotateFromYToZ(angle);
    _height_vector.rotateFromYToZ(angle);
    _depth_vector.rotateFromYToZ(angle);
    return *this;
}

Box& Box::rotateFromZToX(imp_float angle)
{
    _width_vector.rotateFromZToX(angle);
    _height_vector.rotateFromZToX(angle);
    _depth_vector.rotateFromZToX(angle);
    return *this;
}

Box& Box::makeAxesPerpendicular()
{
    _height_vector = _height_vector.getProjectedOnNormalTo(_depth_vector, _width_vector);
    _depth_vector = _depth_vector.getProjectedOnNormalTo(_width_vector, _height_vector);
    _height = _height_vector.getLength();
    _depth = _depth_vector.getLength();
    return *this;
}

Sphere Box::getBoundingSphere() const
{
    const Vector& radius_vector = (_width_vector + _height_vector + _depth_vector)*0.5;
    return Sphere(origin + radius_vector,
                  radius_vector.getLength());
}

AxisAlignedBox Box::getAABB() const
{
    const std::vector<Point>& corners = getCorners();
    Point min_point = corners[0];
    Point max_point = min_point;

    for (imp_uint i = 1; i < 8; i++)
    {
        min_point.useSmallestCoordinates(corners[i]);
        max_point.useLargestCoordinates(corners[i]);
    }

    return AxisAlignedBox(min_point, max_point);
}

} // Geometry3D
} // Impact
