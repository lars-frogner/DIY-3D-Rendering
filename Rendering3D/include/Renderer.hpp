#pragma once
#include "precision.hpp"
#include "Model.hpp"
#include "Material.hpp"
#include "Light.hpp"
#include "Image.hpp"
#include "Point2.hpp"
#include "Vector2.hpp"
#include "Triangle2.hpp"
#include "AxisAlignedRectangle.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"
#include "Camera.hpp"
#include "Ray.hpp"
#include "Plane.hpp"
#include "CoordinateFrame.hpp"
#include "TriangleMesh.hpp"
#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"
#include <vector>
#include <string>

namespace Impact {
namespace Rendering3D {

class Renderer {

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
    typedef Geometry3D::Plane Plane;
    typedef Geometry3D::CoordinateFrame CoordinateFrame;
    typedef Geometry3D::TriangleMesh TriangleMesh;
    typedef Geometry3D::AffineTransformation AffineTransformation;
    typedef Geometry3D::ProjectiveTransformation ProjectiveTransformation;

protected:

    const imp_float _ray_origin_offset = static_cast<imp_float>(1e-4);
	
    Image* _image;
    Camera* _camera;
	std::vector<Light*>* _lights;
	std::vector<Model*>* _models;
	std::vector<CoordinateFrame> _light_world_cframes;

	std::vector<TriangleMesh> _mesh_copies;

	bool _lights_in_camera_system = false;
    
    CoordinateFrame _camera_cframe;
    
    Point2 _image_lower_corner;
    Point2 _image_upper_corner;
    
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

	Plane _frustum_lower_plane;
	Plane _frustum_upper_plane;
	Plane _frustum_left_plane;
	Plane _frustum_right_plane;

    void computeViewData();

	void storeLightWorldCoordinateFrames();

	void transformLightsToCameraSystem();
	void transformLightsToWorldSystem();

	void createTransformedMeshCopies();
	void createTransformedMeshCopies(const AffineTransformation& additional_transformation);

    Ray getEyeRay(const Point2& pixel_center) const;

    Radiance getScatteredRadiance(const Point&  surface_point,
								  const Vector& surface_normal,
								  const Vector& scatter_direction,
								  const Material* material) const;

    bool evaluateSourceVisibility(const Point& surface_point,
								  const Vector& direction_to_source,
								  imp_float distance_to_source) const;

	void computeRadianceAtAllVertices(TriangleMesh& mesh, const Material* material);

	bool computeRadianceAtIntersectedPosition(const TriangleMesh& mesh,
											  const Material* material,
											  const Ray& ray,
											  imp_uint face_idx,
										  	  Radiance& pixel_radiance,
											  imp_float& closest_distance) const;

	void drawFaces(const TriangleMesh& mesh) const;
	void drawFaces(const TriangleMesh& mesh, Color color) const;

	void drawEdges(const TriangleMesh& mesh, float luminance) const;

public:

    imp_uint n_splits = 0;

	bool gamma_encode = true;
    bool use_omp = true;
	bool render_depth_map = false;
	bool renormalize_depth_map = true;
	bool draw_edges = false;

	Color bg_color = Color::black();
    float edge_brightness = 0;

    Renderer(Image* new_image,
			 Camera* new_camera,
			 std::vector<Light*>* new_lights,
			 std::vector<Model*>* new_models);

    Renderer(const Renderer& other) = delete;
	Renderer& operator=(const Renderer& other) = delete;

	void transformCameraLookRay(const AffineTransformation& transformation);

	void initialize();

    void renderDirect();
    void rayTrace();
    void rasterize();

	void saveImage(const std::string& filename);

	const Image& getImage() const;
	const Camera& getCamera() const;

	void toggleModelShadows();
};

} // Rendering3D
} // Impact
