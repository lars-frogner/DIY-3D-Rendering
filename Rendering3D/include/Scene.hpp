#pragma once
#include "precision.hpp"
#include "RenderableTriangleMesh.hpp"
#include "SceneGraph.hpp"
#include "Material.hpp"
#include "Light.hpp"
#include "LightContainer.hpp"
#include "Image.hpp"
#include "BlinnPhongMaterial.hpp"
#include "Point2.hpp"
#include "Vector2.hpp"
#include "Triangle2.hpp"
#include "AxisAlignedRectangle.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"
#include "Camera.hpp"
#include "Ray.hpp"
#include "CoordinateFrame.hpp"
#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"
#include <armadillo>
#include <vector>

namespace Impact {
namespace Rendering3D {

class Scene {

friend RenderableTriangleMesh;

private:
    typedef Geometry2D::Point Point2;
    typedef Geometry2D::Vector Vector2;
    typedef Geometry2D::Triangle Triangle2;
    typedef Geometry2D::AxisAlignedRectangle AxisAlignedRectangle;
    typedef Geometry3D::Point Point;
    typedef Geometry3D::Vector Vector;
    typedef Geometry3D::Vector4 Vector4;
    typedef Geometry3D::Camera Camera;
    typedef Geometry3D::Ray Ray;
    typedef Geometry3D::CoordinateFrame CoordinateFrame;
    typedef Geometry3D::AffineTransformation AffineTransformation;
    typedef Geometry3D::ProjectiveTransformation ProjectiveTransformation;

    const imp_float _ray_origin_offset = static_cast<imp_float>(1e-4);

    Camera _camera;
    Image _image;

	LightContainer _lights;
	std::vector<RenderableTriangleMesh> _objects;
    
    imp_float _image_width, _image_height;
    imp_float _inverse_image_width, _inverse_image_height;
    imp_float _image_width_at_unit_distance_from_camera;
    imp_float _image_height_at_unit_distance_from_camera;
    imp_float _inverse_image_width_at_unit_distance_from_camera;
    imp_float _inverse_image_height_at_unit_distance_from_camera;

    AffineTransformation _transformation_to_camera_system;
    AffineTransformation _transformation_from_camera_system;

    imp_float _near_plane_distance, _far_plane_distance;

	ProjectiveTransformation _perspective_transformation;
	AffineTransformation _windowing_transformation;

    void _computeViewData();

    Ray _getEyeRay(const Point2& pixel_center) const;

    bool _evaluateVisibility(const Point& surface_point,
                             const Vector& direction_to_source,
                             imp_float distance_to_source) const;

protected:
    
    Point _camera_position;
    
    Point2 _image_lower_corner;
    Point2 _image_upper_corner;

    Point2 _getPerspectiveProjected(const arma::Col<imp_float>& vertex) const;

    Radiance _getRadiance(const Point&  surface_point,
                          const Vector& surface_normal,
                          const Vector& scatter_direction,
                          const Material* material) const;

public:

    imp_uint n_splits = 0;
	bool gamma_encode = true;
    bool use_omp = true;

    Color face_color = Color::white();
	Color bg_color = Color::black();
    float edge_brightness = 0.6f;

    Scene(const Camera& new_camera,
          const Image& new_image);

	void transformCameraLookRay(const AffineTransformation& transformation);

	const Camera& getCamera() const;
    const Image& getImage() const;
    
    Scene& generateGround(imp_float plane_height, imp_float horizon_position, const BlinnPhongMaterial& material);

    Scene& renderDirect(const std::vector<RenderableTriangleMesh>& objects, const LightContainer& lights);
    Scene& rayTrace(const std::vector<RenderableTriangleMesh>& objects, const LightContainer& lights);
    Scene& rasterize(const std::vector<RenderableTriangleMesh>& objects, const LightContainer& lights);
};

} // Rendering3D
} // Impact
