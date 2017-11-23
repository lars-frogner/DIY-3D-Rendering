#include "Sphere.hpp"
#include <cmath>

namespace Impact {
namespace Geometry3D {

Sphere::Sphere(const Point& new_center,
               imp_float    new_radius)
    : center(new_center), radius(new_radius) {}

Sphere::Sphere(const Point&  new_center,
               const Vector& radius_vector)
    : center(new_center), radius(radius_vector.getLength()) {}

Sphere::Sphere(const Point& new_center,
               const Point& point_on_surface)
    : center(new_center), radius((point_on_surface - new_center).getLength()) {}

Point Sphere::operator()(imp_float theta, imp_float phi) const
{
    imp_float r_sin_theta = radius*sin(theta);
    return center + Vector(r_sin_theta*cos(phi),
                           r_sin_theta*sin(phi),
                           radius*cos(theta));
}

AxisAlignedBox Sphere::getAABB() const
{
    Vector extent(radius, radius, radius);
    return AxisAlignedBox(center - extent, center + extent);
}

} // Geometry3D
} // Impact
