#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "AxisAlignedBox.hpp"

namespace Impact {
namespace Geometry3D {

class Sphere {

public:
    Point center;
    imp_float radius;

    Sphere(const Point& new_center,
           imp_float    new_radius);

    Sphere(const Point&  new_center,
           const Vector& radius_vector);

    Sphere(const Point& new_center,
           const Point& point_on_surface);

    Point operator()(imp_float theta, imp_float phi) const;

    AxisAlignedBox getAABB() const;
};

} // Geometry3D
} // Impact
