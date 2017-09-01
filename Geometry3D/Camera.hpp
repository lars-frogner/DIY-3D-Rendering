#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "Point.hpp"
#include "Vector.hpp"
#include "Ray.hpp"
#include "CoordinateFrame.hpp"
#include "../Transformations3D/AffineTransformation.hpp"
#include "../Transformations3D/ProjectiveTransformation.hpp"

namespace Geometry3D {

template <typename F>
class Camera {

private:
    typedef Transformations3D::AffineTransformation<F> AffineTransformation;
    typedef Transformations3D::ProjectiveTransformation<F> ProjectiveTransformation;

    const F _DEG_TO_RAD = static_cast<F>(M_PI/180.0);

    Ray<F> _look_ray;
    Vector<F> _up_direction;
    F _near_plane_distance, _far_plane_distance;
    F _field_of_view;

    CoordinateFrame<F> _coordinate_frame;

    CoordinateFrame<F> _getCoordinateFrame() const;

public:

    Camera<F>();
    Camera<F>(const Ray<F>& new_look_ray,
              const Vector<F>& new_up_direction,
              F new_near_plane_distance,
              F new_far_plane_distance,
              F new_field_of_view);
    
    const Point<F>& getPosition() const;
    F getFieldOfView() const;
    F getNearPlaneDistance() const;
    F getFarPlaneDistance() const;
    const CoordinateFrame<F>& getCoordinateFrame() const;

    AffineTransformation getWindowingTransformation(size_t width, F aspect_ratio) const;
    AffineTransformation getWorldToPerspectiveViewVolumeTransformation(F aspect_ratio) const;
    ProjectiveTransformation getPerspectiveToParallelViewVolumeTransformation() const;
    ProjectiveTransformation getWorldToParallelViewVolumeTransformation(F aspect_ratio) const;
};

template <typename F>
Camera<F>::Camera()
    : _look_ray(Point<F>::origin(), Vector<F>(0, 0, -1)),
      _up_direction(0, 1, 0),
      _near_plane_distance(1),
      _far_plane_distance(1000),
      _field_of_view(45*_DEG_TO_RAD) {}

template <typename F>
Camera<F>::Camera(const Ray<F>& new_look_ray,
                  const Vector<F>& new_up_direction,
                  F new_near_plane_distance,
                  F new_far_plane_distance,
                  F new_field_of_view)
    : _look_ray(new_look_ray.origin, new_look_ray.direction.getNormalized()),
      _up_direction(new_up_direction),
      _near_plane_distance(new_near_plane_distance),
      _far_plane_distance(new_far_plane_distance),
      _field_of_view(new_field_of_view*_DEG_TO_RAD),
      _coordinate_frame(_getCoordinateFrame()) {}

template <typename F>
CoordinateFrame<F> Camera<F>::_getCoordinateFrame() const
{
    const Vector<F>& w = -_look_ray.direction;
    const Vector<F>& v = (_up_direction - _up_direction.dot(w)*w).normalize();
    const Vector<F>& u = v.cross(w);

    return CoordinateFrame<F>(_look_ray.origin, u, v, w);
}

template <typename F>
const Point<F>& Camera<F>::getPosition() const
{
    return _look_ray.origin;
}

template <typename F>
F Camera<F>::getFieldOfView() const
{
    return _field_of_view;
}

template <typename F>
F Camera<F>::getNearPlaneDistance() const
{
    return _near_plane_distance;
}

template <typename F>
F Camera<F>::getFarPlaneDistance() const
{
    return _far_plane_distance;
}

template <typename F>
const CoordinateFrame<F>& Camera<F>::getCoordinateFrame() const
{
    return _coordinate_frame;
}

template <typename F>
Transformations3D::AffineTransformation<F> Camera<F>::getWindowingTransformation(size_t width, F aspect_ratio) const
{
    F width_float = static_cast<F>(width);
    F height_float = width_float/aspect_ratio;
    return AffineTransformation::windowing(width_float-1, height_float-1);
}

template <typename F>
Transformations3D::AffineTransformation<F> Camera<F>::getWorldToPerspectiveViewVolumeTransformation(F aspect_ratio) const
{
    
    F horizontal_angle = _field_of_view/2;
    F far_plane_halfwidth = _far_plane_distance*tan(horizontal_angle);
    F far_plane_halfheight = far_plane_halfwidth/aspect_ratio;

    const Point<F>& P = _look_ray.origin;
    const Point<F>& A = P - _far_plane_distance*_coordinate_frame.basis_3;
    const Point<F>& B = A + far_plane_halfwidth*_coordinate_frame.basis_1;
    const Point<F>& C = A + far_plane_halfheight*_coordinate_frame.basis_2;

    return AffineTransformation::pointsToPoints(P, A, B, C,
                                                Point<F>::origin(), Point<F>(0, 0, -1),
                                                Point<F>(1, 0, -1), Point<F>(0, 1, -1));
}

template <typename F>
Transformations3D::ProjectiveTransformation<F> Camera<F>::getPerspectiveToParallelViewVolumeTransformation() const
{
    return ProjectiveTransformation::unhinging(_near_plane_distance, _far_plane_distance);
}

template <typename F>
Transformations3D::ProjectiveTransformation<F> Camera<F>::getWorldToParallelViewVolumeTransformation(F aspect_ratio) const
{
    return getPerspectiveToParallelViewVolumeTransformation()*getWorldToPerspectiveViewVolumeTransformation(aspect_ratio);
}

} // Geometry3D
