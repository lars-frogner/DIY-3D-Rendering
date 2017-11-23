#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Ray.hpp"
#include "CoordinateFrame.hpp"
#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"

namespace Impact {
namespace Geometry3D {

class Camera {

private:
    Ray _look_ray;
    Vector _up_direction;
    imp_float _near_plane_distance, _far_plane_distance;
    imp_float _field_of_view;

    CoordinateFrame _coordinate_frame;

    CoordinateFrame _getCoordinateFrame() const;

public:

    Camera();
    Camera(const Ray& new_look_ray,
           const Vector& new_up_direction,
           imp_float new_near_plane_distance,
           imp_float new_far_plane_distance,
           imp_float new_field_of_view);

	void transformLookRay(const AffineTransformation& transformation);
    
    const Point& getPosition() const;
	const Vector& getLookDirection() const;
    imp_float getFieldOfView() const;
    imp_float getNearPlaneDistance() const;
    imp_float getFarPlaneDistance() const;
    const CoordinateFrame& getCoordinateFrame() const;

    AffineTransformation getWindowingTransformation(imp_uint width, imp_float aspect_ratio) const;
    AffineTransformation getWorldToPerspectiveViewVolumeTransformation(imp_float aspect_ratio) const;
    ProjectiveTransformation getPerspectiveToParallelViewVolumeTransformation() const;
    ProjectiveTransformation getWorldToParallelViewVolumeTransformation(imp_float aspect_ratio) const;
};

} // Geometry3D
} // Impact
