#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Geometry3D {

class Ray {

public:
    Point origin;
    Vector direction;
    Vector inverse_direction;
    imp_float max_distance;

    Ray(const Point&  new_origin,
        const Vector& new_direction);

    Ray(const Point&  new_origin,
        const Vector& new_direction,
        imp_float new_max_distance);

    Point operator()(imp_float distance) const;

    Ray& alignWith(const Vector& other_direction);
    Ray& pointAt(const Point& point);
};

} // Geometry3D
} // Impact
