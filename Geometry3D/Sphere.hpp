#pragma once
#include <math.h>
#include "Point.hpp"
#include "Vector.hpp"
#include "AxisAlignedBox.hpp"

namespace Geometry3D {

template <typename F>
class Sphere {

public:
    Point<F> center;
    F radius;

    Sphere<F>(const Point<F>& new_center,
              F				  new_radius);

    Sphere<F>(const Point<F>&  new_center,
              const Vector<F>& radius_vector);

    Sphere<F>(const Point<F>& new_center,
              const Point<F>& point_on_surface);

    Point<F> operator()(F theta, F phi) const;

    AxisAlignedBox<F> getAABB() const;
};

template <typename F>
Sphere<F>::Sphere(const Point<F>& new_center,
                  F				  new_radius)
    : center(new_center), radius(new_radius) {}

template <typename F>
Sphere<F>::Sphere(const Point<F>&  new_center,
                  const Vector<F>& radius_vector)
    : center(new_center), radius(radius_vector.getLength()) {}

template <typename F>
Sphere<F>::Sphere(const Point<F>& new_center,
                  const Point<F>& point_on_surface)
    : center(new_center), radius((point_on_surface - new_center).getLength()) {}

template <typename F>
Point<F> Sphere<F>::operator()(F theta, F phi) const
{
    F r_sin_theta = radius*sin(theta);
    return center + Vector<F>(r_sin_theta*cos(phi),
                              r_sin_theta*sin(phi),
                              radius*cos(theta));
}

template <typename F>
AxisAlignedBox<F> Sphere<F>::getAABB() const
{
    Vector<F> extent(radius, radius, radius);
    return AxisAlignedBox<F>(center - extent, center + extent);
}

} // Geometry3D
