#pragma once
#include <omp.h>
#include <vector>
#include <limits>
#include "../Geometry2D/Point.hpp"
#include "../Geometry2D/Vector.hpp"
#include "../Geometry2D/Triangle.hpp"
#include "../Geometry2D/AxisAlignedRectangle.hpp"
#include "../Geometry3D/Point.hpp"
#include "../Geometry3D/Vector.hpp"
#include "../Geometry3D/Vector4.hpp"
#include "../Geometry3D/TriangleMesh.hpp"
#include "../Geometry3D/Camera.hpp"
#include "../Geometry3D/Ray.hpp"
#include "../Geometry3D/CoordinateFrame.hpp"
#include "../Transformations3D/AffineTransformation.hpp"
#include "../Transformations3D/ProjectiveTransformation.hpp"
#include "SceneGraph.hpp"
#include "Material.hpp"
#include "Light.hpp"
#include "LightContainer.hpp"
#include "Image.hpp"
#include "BlinnPhongMaterial.hpp"

namespace Rendering3D {

template <typename F>
class Scene {

friend Geometry3D::TriangleMesh<F>;

private:
    typedef Geometry2D::Point<F> Point2;
    typedef Geometry2D::Vector<F> Vector2;
    typedef Geometry2D::Triangle<F> Triangle2;
    typedef Geometry2D::AxisAlignedRectangle<F> AxisAlignedRectangle;
    typedef Geometry3D::Point<F> Point;
    typedef Geometry3D::Vector<F> Vector;
    typedef Geometry3D::Vector4<F> Vector4;
    typedef Geometry3D::TriangleMesh<F> TriangleMesh;
    typedef Geometry3D::Camera<F> Camera;
    typedef Geometry3D::Ray<F> Ray;
    typedef Geometry3D::CoordinateFrame<F> CoordinateFrame;
    typedef Transformations3D::AffineTransformation<F> AffineTransformation;
    typedef Transformations3D::ProjectiveTransformation<F> ProjectiveTransformation;

    const F _INFINITY = std::numeric_limits<F>::infinity();

    std::vector<TriangleMesh> _objects;
    LightContainer<F> _lights;
    Camera _camera;
    Image<F> _image;
    
    F _image_width, _image_height;
    F _inverse_image_width, _inverse_image_height;
    F _image_width_at_unit_distance_from_camera;
    F _image_height_at_unit_distance_from_camera;
    F _inverse_image_width_at_unit_distance_from_camera;
    F _inverse_image_height_at_unit_distance_from_camera;

    AffineTransformation _transformation_to_camera_system;
    AffineTransformation _transformation_from_camera_system;

    F _near_plane_distance, _far_plane_distance;

    void _setConstants();

    Ray _getEyeRay(F x, F y) const;

    bool _evaluateVisibility(const Point& surface_point,
                             const Vector& direction_to_source,
                             F distance_to_source) const;

protected:
    
    Point _camera_position;

    Point2 _getPerspectiveProjected(const arma::Col<F>& vertex) const;

    Radiance _getRadiance(const Point&  surface_point,
                          const Vector& surface_normal,
                          const Vector& scatter_direction,
                          const Material<F>* material,
                          bool ignore_visibility) const;

public:

    size_t n_splits = 0;
    bool use_direct_lighting = true;
    bool remove_hidden_faces = true;
    bool perform_clipping = true;
    bool draw_faces = true;
    bool draw_edges = false;
    bool use_omp = true;

    Color face_color = Color::white();
    float edge_brightness = 0.0f;

    Scene<F>(SceneGraph<F>& scene_graph,
             const LightContainer<F>& new_lights,
             const Camera& new_camera,
             const Image<F>& new_image);

    const Image<F>& getImage() const;
    
    Scene<F>& generateGround(F plane_height, F horizon_position, const BlinnPhongMaterial<F>& material);

    Scene<F>& renderDirect();
    Scene<F>& rayTrace();
    Scene<F>& rasterize();
};

template <typename F>
Scene<F>::Scene(SceneGraph<F>& scene_graph,
                const LightContainer<F>& new_lights,
                const Camera& new_camera,
                const Image<F>& new_image)
    : _objects(SceneGraph<F>::getTransformedObjects(scene_graph)),
      _lights(new_lights),
      _camera(new_camera),
      _image(new_image)
{
    _setConstants();
}

template <typename F>
void Scene<F>::_setConstants()
{
    _image_width = static_cast<F>(_image.getWidth());
    _image_height = static_cast<F>(_image.getHeight());

    _inverse_image_width = 1/_image_width;
    _inverse_image_height = 1/_image_height;

    _image_width_at_unit_distance_from_camera = 2*tan(_camera.getFieldOfView()/2);
    _image_height_at_unit_distance_from_camera = _image_width_at_unit_distance_from_camera/_image.getAspectRatio();

    _inverse_image_width_at_unit_distance_from_camera = 1/_image_width_at_unit_distance_from_camera;
    _inverse_image_height_at_unit_distance_from_camera = 1/_image_height_at_unit_distance_from_camera;

    _transformation_to_camera_system = AffineTransformation::toCoordinateFrame(_camera.getCoordinateFrame());
    _transformation_from_camera_system = _transformation_to_camera_system.getInverse();

    _near_plane_distance = _camera.getNearPlaneDistance();
    _far_plane_distance = _camera.getFarPlaneDistance();
}

template <typename F>
const Image<F>& Scene<F>::getImage() const
{
    return _image;
}

template <typename F>
Scene<F>& Scene<F>::renderDirect()
{
    const ProjectiveTransformation& perspective_transformation = _camera.getWorldToParallelViewVolumeTransformation(_image.getAspectRatio());
    const AffineTransformation& windowing_transformation = _camera.getWindowingTransformation(_image.getWidth(), _image.getAspectRatio());

    _image.initializeDepthBuffer(-1);

    for (size_t i = 0; i < _objects.size(); i++)
    {
        TriangleMesh& mesh = _objects[i];

        if (n_splits > 0)
            mesh.splitFaces(n_splits);

        if (use_direct_lighting)
            mesh.computeNormals()
                .shadeVerticesDirect(*this);

        mesh.applyTransformation(perspective_transformation)
            .homogenizeVertices();
        
        if (remove_hidden_faces)
            mesh.removeBackwardFacingFaces();
        
        if (perform_clipping)
            mesh.computeAABB()
                .performClipping();

        mesh.applyWindowingTransformation(windowing_transformation);

        if (draw_faces)
            if (use_direct_lighting)
                mesh.drawFaces(_image);
            else
                mesh.drawFaces(_image, face_color);

        if (draw_edges)
            mesh.drawEdges(_image, edge_brightness);
    }

    return *this;
}

template <typename F>
Scene<F>& Scene<F>::rayTrace()
{
    size_t image_width = _image.getWidth();
    size_t image_height = _image.getHeight();
    size_t n_objects = _objects.size();
    size_t n_triangles;
    int x, y, obj_idx, face_idx;
    F closest_distance;
    Radiance pixel_radiance;
    
    #pragma omp parallel for default(shared) \
                             private(obj_idx) \
                             shared(n_objects) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
        _objects[obj_idx].applyTransformation(_transformation_to_camera_system)
                         .computeNormals()
                         .computeAABB();
    }

    _lights.applyTransformation(_transformation_to_camera_system);
    
    #pragma omp parallel for default(shared) \
                             private(x, y, obj_idx, face_idx, n_triangles, closest_distance, pixel_radiance) \
                             shared(image_height, image_width, n_objects) \
                             if (use_omp)
    for (y = 0; y < image_height; y++) {
        for (x = 0; x < image_width; x++)
        {
            const Ray& eye_ray = _getEyeRay(static_cast<F>(x) + 0.5f,
                                            static_cast<F>(y) + 0.5f);
                
            closest_distance = _far_plane_distance;

            for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
            {
                const TriangleMesh& mesh = _objects[obj_idx];

                if (mesh.evaluateRayAABBIntersection(eye_ray, _far_plane_distance))
                {
                    n_triangles = mesh.getNumberOfFaces();

                    for (face_idx = 0; face_idx < n_triangles; face_idx++)
                    {
                        if (mesh.sampleRayTriangle(*this, x, y, eye_ray, face_idx, pixel_radiance, closest_distance))
                        {
                            _image.setRadiance(x, y, pixel_radiance);
                        }
                    }
                }
            }
        }
    }

    _image.gammaEncode(1.0f);

    return *this;
}

template <typename F>
Scene<F>& Scene<F>::rasterize()
{
    size_t n_objects = _objects.size();
    size_t n_triangles;
    int x, y, obj_idx, face_idx, vertex_idx;
    int x_min, x_max, y_min, y_max;

    Point2 image_lower_corner = Point2::origin();
    Point2 image_upper_corner(_image_width, _image_height);

    F alpha, beta, gamma;
    Point2 pixel_center;
    Point vertices[3];
    Vector normals[3];
    Material<F>* material = NULL;
    
    F inverse_depths[3];
    F interpolated_inverse_depth;
    F depth;

    _image.initializeDepthBuffer(_far_plane_distance);
    
    #pragma omp parallel for default(shared) \
                             private(obj_idx) \
                             shared(n_objects) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
        _objects[obj_idx].applyTransformation(_transformation_to_camera_system)
                         .computeNormals()
                         .clipNearPlaneAt(-_near_plane_distance)
                         .computeAABB();
    }

    _lights.applyTransformation(_transformation_to_camera_system);
        
    #pragma omp parallel for default(shared) \
                             private(x, y, obj_idx, face_idx, vertex_idx, n_triangles, x_min, x_max, y_min, y_max, pixel_center, alpha, beta, gamma, inverse_depths, vertices, normals, material, interpolated_inverse_depth, depth) \
                             shared(n_objects, image_lower_corner, image_upper_corner) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
        const TriangleMesh& mesh = _objects[obj_idx];

        n_triangles = mesh.getNumberOfFaces();

        for (face_idx = 0; face_idx < n_triangles; face_idx++)
        {
            Triangle2& projected_triangle = mesh.getProjectedFace(*this, face_idx);

            const AxisAlignedRectangle& aabb = projected_triangle.getAABB(image_lower_corner, image_upper_corner);

            x_min = static_cast<int>(aabb.lower_corner.x + 0.5f);
            x_max = static_cast<int>(aabb.upper_corner.x + 0.5f);
            y_min = static_cast<int>(aabb.lower_corner.y + 0.5f);
            y_max = static_cast<int>(aabb.upper_corner.y + 0.5f);

            projected_triangle.precomputeBarycentricQuantities();

            mesh.getVertexAttributes(face_idx, vertices, normals, material);

            for (vertex_idx = 0; vertex_idx < 3; vertex_idx++)
            {
                inverse_depths[vertex_idx] = -1.0f/vertices[vertex_idx].z;
                vertices[vertex_idx] *= inverse_depths[vertex_idx];
                normals[vertex_idx] *= inverse_depths[vertex_idx];
            }

            for (y = y_min; y < y_max; y++) {
                for (x = x_min; x < x_max; x++)
                {
                    pixel_center.moveTo(static_cast<F>(x) + 0.5f, static_cast<F>(y) + 0.5f);

                    projected_triangle.getBarycentricCoordinates(pixel_center, alpha, beta, gamma);

                    if (alpha > 0 && beta > 0 && gamma > 0)
                    {
                        interpolated_inverse_depth = alpha*inverse_depths[0] + beta*inverse_depths[1] + gamma*inverse_depths[2];

                        const Point& surface_point = (vertices[0] + (vertices[1] - vertices[0])*beta + (vertices[2] - vertices[0])*gamma)/interpolated_inverse_depth;
                        const Vector& surface_normal = ((normals[0]*alpha + normals[1]*beta + normals[2]*gamma)/interpolated_inverse_depth).normalize();

                        const Vector& displacement_to_camera = -(surface_point.toVector());

                        depth = displacement_to_camera.getLength();

                        if (depth < _image.getDepth(x, y))
                        {
                            const Radiance& radiance = _getRadiance(surface_point,
                                                                    surface_normal,
                                                                    displacement_to_camera/depth,
                                                                    material,
                                                                    false);

                            #pragma omp critical
                            if (depth < _image.getDepth(x, y))
                            {
                                _image.setDepth(x, y, depth);
                                _image.setRadiance(x, y, radiance);
                            }
                        }
                    }
                }
            }

            /*_image.setRadiance(x_min, y_min, Radiance::white());
            _image.setRadiance(x_max-1, y_min, Radiance::white());
            _image.setRadiance(x_max-1, y_max-1, Radiance::white());
            _image.setRadiance(x_min, y_max-1, Radiance::white());*/
        }
    }

    _image.gammaEncode(1.0f);

    return *this;
}

template <typename F>
Scene<F>& Scene<F>::generateGround(F plane_height, F horizon_position, const BlinnPhongMaterial<F>& material)
{
    assert(horizon_position >= 0 && horizon_position < 1);

    const Ray& back_left_ray = _transformation_from_camera_system*_getEyeRay(0, horizon_position*_image_height);
    const Ray& back_right_ray = _transformation_from_camera_system*_getEyeRay(_image_width, horizon_position*_image_height);
    const Ray& front_center_ray = _transformation_from_camera_system*_getEyeRay(_image_width/2, 0);

    F back_left_ray_distance = (plane_height - back_left_ray.origin.y)/back_left_ray.direction.y;
    F back_right_ray_distance = (plane_height - back_right_ray.origin.y)/back_right_ray.direction.y;
    F front_center_ray_distance = (plane_height - front_center_ray.origin.y)/front_center_ray.direction.y;

    assert(back_left_ray_distance >= 0 && back_right_ray_distance >= 0 && front_center_ray_distance >= 0);

    const Point& back_left = back_left_ray(back_left_ray_distance);
    const Point& back_right = back_right_ray(back_right_ray_distance);
    const Point& front_center = front_center_ray(front_center_ray_distance);

    const Vector& width_vector = back_right - back_left;
    const Vector& depth_vector = (back_left + width_vector/2) - front_center;
    const Point& front_left = front_center - width_vector/2;

    TriangleMesh& ground_mesh = TriangleMesh::sheet(front_left, width_vector, depth_vector);

    _objects.push_back(ground_mesh.withMaterial(material));

    return *this;
}

template <typename F>
Geometry3D::Ray<F> Scene<F>::_getEyeRay(F x, F y) const
{
    Vector unit_vector_from_camera_origin_to_pixel((x*_inverse_image_width - 0.5f)*_image_width_at_unit_distance_from_camera,
                                                   (y*_inverse_image_height - 0.5f)*_image_height_at_unit_distance_from_camera,
                                                   -1);

    unit_vector_from_camera_origin_to_pixel.normalize();

    return Ray((unit_vector_from_camera_origin_to_pixel*_near_plane_distance).toPoint(), unit_vector_from_camera_origin_to_pixel);
}

template <typename F>
Geometry2D::Point<F> Scene<F>::_getPerspectiveProjected(const arma::Col<F>& vertex_vector) const
{
    F normalization = -1.0f/vertex_vector(2);

    return Point2(_image_width*(vertex_vector(0)*normalization*_inverse_image_width_at_unit_distance_from_camera + 0.5f),
                  _image_height*(vertex_vector(1)*normalization*_inverse_image_height_at_unit_distance_from_camera + 0.5f));
}

template <typename F>
Radiance Scene<F>::_getRadiance(const Point&  surface_point,
                                const Vector& surface_normal,
                                const Vector& scatter_direction,
                                const Material<F>* material,
                                bool ignore_visibility) const
{
    Radiance radiance = Color::black();

    size_t n_lights = _lights.getNumberOfLights();
    size_t n_samples;
    size_t n, s;
    Vector direction_to_source;
    float normalization_factor;
    F distance_to_source;

    for (n = 0; n < n_lights; n++)
    {
        const Light<F>* light = _lights.getLight(n);
        
        n_samples = light->getNumberOfSamples();
        normalization_factor = 0.0f;

        for (s = 0; s < n_samples; s++)
        {
            const Vector4& source_point = light->getRandomPoint();

            if (source_point.w == 0)
            {
                direction_to_source = source_point.toVector();
                distance_to_source = _INFINITY;
            }
            else
            {
                direction_to_source = source_point.getXYZ() - surface_point;
                distance_to_source = direction_to_source.getLength();
                direction_to_source /= distance_to_source;
            }

            if (ignore_visibility || _evaluateVisibility(surface_point, direction_to_source, distance_to_source))
            {
                const Biradiance& biradiance = light->getBiradiance(source_point, surface_point);
                const Color& scattering_density = material->getScatteringDensity(surface_normal, direction_to_source, scatter_direction);
            
                radiance += surface_normal.dot(direction_to_source)*scattering_density*biradiance;
                normalization_factor += 1.0f;
            }
        }

        if (normalization_factor > 0.0f) radiance /= normalization_factor;
    }

    return radiance;
}

template <typename F>
bool Scene<F>::_evaluateVisibility(const Point& surface_point,
                                   const Vector& direction_to_source,
                                   F distance_to_source) const
{
    const F ray_origin_offset = 1e-4f;
    
    Ray shadow_ray(surface_point + direction_to_source*ray_origin_offset, direction_to_source);

    size_t n_objects = _objects.size();
    size_t n_triangles;
    size_t obj_idx, face_idx;
    F alpha, beta, gamma;

    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
        const TriangleMesh& mesh = _objects[obj_idx];

        if (mesh.evaluateRayAABBIntersection(shadow_ray, _INFINITY))
        {
            n_triangles = mesh.getNumberOfFaces();

            for (face_idx = 0; face_idx < n_triangles; face_idx++)
            {
                if (mesh.evaluateRayFaceIntersection(shadow_ray, face_idx, alpha, beta, gamma) < distance_to_source)
                {
                    return false;
                }
            }
        }
    }

    return true;
}

} // Rendering3D

/*
transformation to cframe
transform lights
modify eye ray and projected vertex*/