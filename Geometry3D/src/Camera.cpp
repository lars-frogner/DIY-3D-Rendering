#include "Camera.hpp"
#include <cmath>

namespace Impact {
namespace Geometry3D {

Camera::Camera()
    : _look_ray(Point::origin(), Vector(0, 0, -1)),
      _up_direction(0, 1, 0),
      _near_plane_distance(1),
      _far_plane_distance(100),
      _field_of_view(45*IMP_DEG_TO_RAD),
      _coordinate_frame(getCoordinateFrame(_look_ray.origin, _look_ray.direction, _up_direction)) {}

Camera::Camera(const Ray& new_look_ray,
               const Vector& new_up_direction,
               imp_float new_near_plane_distance,
               imp_float new_far_plane_distance,
               imp_float new_field_of_view)
    : _look_ray(new_look_ray.origin, new_look_ray.direction.getNormalized()),
      _up_direction(new_up_direction),
      _near_plane_distance(new_near_plane_distance),
      _far_plane_distance(new_far_plane_distance),
      _field_of_view(new_field_of_view*IMP_DEG_TO_RAD),
      _coordinate_frame(getCoordinateFrame(_look_ray.origin, _look_ray.direction, _up_direction)) {}

CoordinateFrame Camera::getCoordinateFrame(const Point& position,
										   const Vector& look_direction,
										   const Vector& up_direction)
{
    const Vector& w = -look_direction;
    const Vector& v = (up_direction - up_direction.dot(w)*w).normalize();
    const Vector& u = v.cross(w);

    return CoordinateFrame(position, u, v, w);
}

void Camera::transformLookRay(const AffineTransformation& transformation)
{
	_look_ray = transformation*_look_ray;
	_coordinate_frame = getCoordinateFrame(_look_ray.origin, _look_ray.direction, _up_direction);
}

const Point& Camera::getPosition() const
{
    return _look_ray.origin;
}

const Vector& Camera::getLookDirection() const
{
	return _look_ray.direction;
}

const Vector& Camera::getUpDirection() const
{
	return _up_direction;
}

imp_float Camera::getFieldOfView() const
{
    return _field_of_view;
}

imp_float Camera::getNearPlaneDistance() const
{
    return _near_plane_distance;
}

imp_float Camera::getFarPlaneDistance() const
{
    return _far_plane_distance;
}

const CoordinateFrame& Camera::getCoordinateFrame() const
{
    return _coordinate_frame;
}

AffineTransformation Camera::getWindowingTransformation(imp_uint width, imp_float aspect_ratio) const
{
    imp_float width_float = static_cast<imp_float>(width);
    imp_float height_float = width_float/aspect_ratio;
    return AffineTransformation::windowing(width_float-1, height_float-1);
}

AffineTransformation Camera::getWorldToPerspectiveViewVolumeTransformation(imp_float aspect_ratio) const
{
    
    imp_float horizontal_angle = _field_of_view/2;
    imp_float far_plane_halfwidth = _far_plane_distance*tan(horizontal_angle);
    imp_float far_plane_halfheight = far_plane_halfwidth/aspect_ratio;

    const Point& P = _look_ray.origin;
    const Point& A = P - _far_plane_distance*_coordinate_frame.basis_3;
    const Point& B = A + far_plane_halfwidth*_coordinate_frame.basis_1;
    const Point& C = A + far_plane_halfheight*_coordinate_frame.basis_2;

    return AffineTransformation::pointsToPoints(P, A, B, C,
                                                Point::origin(), Point(0, 0, -1),
                                                Point(1, 0, -1), Point(0, 1, -1));
}

ProjectiveTransformation Camera::getPerspectiveToParallelViewVolumeTransformation() const
{
    return ProjectiveTransformation::unhinging(_near_plane_distance, _far_plane_distance);
}

ProjectiveTransformation Camera::getWorldToParallelViewVolumeTransformation(imp_float aspect_ratio) const
{
    return getPerspectiveToParallelViewVolumeTransformation()*getWorldToPerspectiveViewVolumeTransformation(aspect_ratio);
}

} // Geometry3D
} // Impact
