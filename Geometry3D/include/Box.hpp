#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Sphere.hpp"
#include "AxisAlignedBox.hpp"
#include <vector>

namespace Impact {
namespace Geometry3D {

class Box {

private:
    Vector _width_vector, _height_vector, _depth_vector;
    imp_float _width, _height, _depth;

public:
    Point origin;

    Box(const Point&  new_origin,
        const Vector& new_width_vector,
        const Vector& new_height_vector,
        const Vector& new_depth_vector);
    
    Box(const Point& new_origin,
        imp_float new_width, imp_float new_height, imp_float new_depth);

    Box(const Point& new_origin,
        const Point& width_point,
        const Point& height_point,
        const Point& depth_point);

    const Vector& getWidthVector() const;
    const Vector& getHeightVector() const;
    const Vector& getDepthVector() const;
    imp_float getWidth() const;
    imp_float getHeight() const;
    imp_float getDepth() const;
    Point getCenter() const;
    std::vector<Point> getCorners() const;

    Box& setWidth(imp_float new_width);
    Box& setHeight(imp_float new_height);
    Box& setDepth(imp_float new_depth);
    Box& setDimensions(imp_float new_width, imp_float new_height, imp_float new_depth);
    Box& setCenter(const Point& center);

    Box& rotateFromXToY(imp_float angle);
    Box& rotateFromYToZ(imp_float angle);
    Box& rotateFromZToX(imp_float angle);

    Box& makeAxesPerpendicular();

    Sphere getBoundingSphere() const;
    AxisAlignedBox getAABB() const;
};

} // Geometry3D
} // Impact
