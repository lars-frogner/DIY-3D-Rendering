#include "Scene.hpp"
#include <cassert>
#include <omp.h>

namespace Impact {
namespace Rendering3D {

Scene::Scene(const Camera& new_camera,
             const Image& new_image)
    : _camera(new_camera),
      _image(new_image)
{
	_computeViewData();
}

void Scene::_computeViewData()
{
    _image_width = static_cast<imp_float>(_image.getWidth());
    _image_height = static_cast<imp_float>(_image.getHeight());
    
    _image_lower_corner.moveTo(0, 0);
    _image_upper_corner.moveTo(_image_width - 1, _image_height - 1);

    _inverse_image_width = 1/_image_width;
    _inverse_image_height = 1/_image_height;

    _image_width_at_unit_distance_from_camera = 2*tan(_camera.getFieldOfView()/2);
    _image_height_at_unit_distance_from_camera = _image_width_at_unit_distance_from_camera/_image.getAspectRatio();

    _inverse_image_width_at_unit_distance_from_camera = 1/_image_width_at_unit_distance_from_camera;
    _inverse_image_height_at_unit_distance_from_camera = 1/_image_height_at_unit_distance_from_camera;

	_camera_position = _camera.getPosition();

    _transformation_to_camera_system = AffineTransformation::toCoordinateFrame(_camera.getCoordinateFrame());
    _transformation_from_camera_system = _transformation_to_camera_system.getInverse();

    _near_plane_distance = _camera.getNearPlaneDistance();
    _far_plane_distance = _camera.getFarPlaneDistance();

	_perspective_transformation = _camera.getWorldToParallelViewVolumeTransformation(_image.getAspectRatio());
	_windowing_transformation = _camera.getWindowingTransformation(_image.getWidth(), _image.getAspectRatio());
}

void Scene::transformCameraLookRay(const AffineTransformation& transformation)
{
	_camera.transformLookRay(transformation);
	_computeViewData();
}

const Geometry3D::Camera& Scene::getCamera() const
{
	return _camera;
}

const Image& Scene::getImage() const
{
    return _image;
}

Scene& Scene::renderDirect(const std::vector<RenderableTriangleMesh>& objects, const LightContainer& lights)
{
	int n_objects = static_cast<int>(objects.size());
	int obj_idx;

	_objects = objects;
	_lights = LightContainer(lights);

	_image.use_omp = use_omp;
	
	_image.setBackgroundColor(bg_color);
    _image.initializeDepthBuffer(-1.0);
	
    #pragma omp parallel for default(shared) \
                             private(obj_idx) \
                             shared(n_objects) \
                             schedule(dynamic) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
        RenderableTriangleMesh& mesh = _objects[obj_idx];

		mesh.uses_omp = use_omp;

        if (n_splits > 0)
            mesh.splitFaces(n_splits);

		if (mesh.casts_shadows)
		{
			mesh.computeAABB();
			mesh.computeBoundingVolumeHierarchy();
		}
	}

    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
        RenderableTriangleMesh& mesh = _objects[obj_idx];

        if (mesh.uses_direct_lighting)
            mesh.shadeVerticesDirect(*this);
	}

    #pragma omp parallel for default(shared) \
                             private(obj_idx) \
                             shared(n_objects) \
                             schedule(dynamic) \
                             if (use_omp)
	for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
	{
		RenderableTriangleMesh& mesh = _objects[obj_idx];

		mesh.applyTransformation(_perspective_transformation);

		if (mesh.perform_clipping && mesh.allZAbove(0))
		{
			mesh.is_visible = false;
			continue;
		}
		
		if (mesh.perform_clipping)
			mesh.clipNearPlane();

		mesh.homogenizeVertices();

		if (mesh.remove_hidden_faces)
			mesh.removeBackwardFacingFaces();

		if (mesh.perform_clipping)
		{
			mesh.computeAABB();

			if (mesh.isInsideParallelViewVolume())
			{
				mesh.clipNonNearPlanes();
			}
			else
			{
				mesh.is_visible = false;
				continue;
			}
		}

		mesh.applyWindowingTransformation(_windowing_transformation);
	}

	for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
	{
		const RenderableTriangleMesh& mesh = _objects[obj_idx];

		if (!mesh.is_visible)
			continue;

        if (mesh.render_faces)
            if (mesh.uses_direct_lighting)
                mesh.drawFaces(_image);
            else
                mesh.drawFaces(_image, face_color);

        if (mesh.render_edges)
            mesh.drawEdges(_image, edge_brightness);
    }

	if (gamma_encode)
		_image.gammaEncodeApprox(1.0f);

	_objects.clear();

    return *this;
}

Scene& Scene::rayTrace(const std::vector<RenderableTriangleMesh>& objects, const LightContainer& lights)
{
    int image_width = static_cast<int>(_image.getWidth());
    int image_height = static_cast<int>(_image.getHeight());
    int n_objects = static_cast<int>(objects.size());
    int x, y, obj_idx;
    std::vector<imp_uint>::const_iterator iter;
    imp_float closest_distance;
    Point2 pixel_center;
    Radiance pixel_radiance;

	_objects = objects;
	_lights = LightContainer(lights);

	_image.use_omp = use_omp;

	_image.setBackgroundColor(bg_color);
    
    #pragma omp parallel for default(shared) \
                             private(obj_idx) \
                             shared(n_objects) \
                             schedule(dynamic) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
        RenderableTriangleMesh& mesh = _objects[obj_idx];
        mesh.applyTransformation(_transformation_to_camera_system);

		mesh.computeNormals();

		if (mesh.perform_clipping && mesh.allZAbove(-_near_plane_distance))
		{
			mesh.is_visible = false;

			if (mesh.casts_shadows)
			{
				mesh.computeAABB();
				mesh.computeBoundingVolumeHierarchy();
			}

			continue;
		}

		if (mesh.perform_clipping)
	        mesh.clipNearPlaneAt(-_near_plane_distance);

		mesh.computeAABB();
		mesh.computeBoundingAreaHierarchy(*this);

		if (mesh.casts_shadows)
			mesh.computeBoundingVolumeHierarchy();

		mesh.is_visible = true;
    }

    _lights.applyTransformation(_transformation_to_camera_system);
    
    #pragma omp parallel for default(shared) \
                             private(x, y, pixel_center, obj_idx, iter, closest_distance, pixel_radiance) \
                             shared(image_height, image_width, n_objects) \
                             schedule(dynamic) \
                             if (use_omp)
    for (y = 0; y < image_height; y++) {
        for (x = 0; x < image_width; x++)
        {
            pixel_center.moveTo(static_cast<imp_float>(x) + 0.5f, static_cast<imp_float>(y) + 0.5f);

            const Ray& eye_ray = _getEyeRay(pixel_center);
                
            closest_distance = _far_plane_distance;

            for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
            {
                const RenderableTriangleMesh& mesh = _objects[obj_idx];

				if (mesh.is_visible)
				{
					const std::vector<imp_uint>& intersected_faces = mesh.getIntersectedFaceIndices(pixel_center);

					for (iter = intersected_faces.begin(); iter != intersected_faces.end(); ++iter)
					{
						if (mesh.sampleRadianceFromFace(*this, eye_ray, *iter, pixel_radiance, closest_distance))
						{
							_image.setRadiance(x, y, pixel_radiance);
						}
					}
				}
            }
        }
    }
    
    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
		const RenderableTriangleMesh& mesh = _objects[obj_idx];

		if (mesh.render_edges)
			mesh.drawEdges(_image, edge_brightness);

		//mesh.drawBoundingAreaHierarchy(_image, edge_brightness);
    }

	if (gamma_encode)
		_image.gammaEncodeApprox(1.0f);

	_objects.clear();

    return *this;
}

Scene& Scene::rasterize(const std::vector<RenderableTriangleMesh>& objects, const LightContainer& lights)
{

	int n_objects = static_cast<int>(objects.size());
    int n_triangles;
    int x, y, obj_idx, face_idx, vertex_idx;
    int x_min, x_max, y_min, y_max;

    imp_float alpha, beta, gamma;
    Point2 pixel_center;
    Point vertices[3];
    Vector normals[3];
    Material* material = NULL;
    
    imp_float inverse_depths[3];
    imp_float interpolated_inverse_depth;
    imp_float depth;

	_objects = objects;
	_lights = LightContainer(lights);

	_image.use_omp = use_omp;

	_image.setBackgroundColor(bg_color);
    _image.initializeDepthBuffer(_far_plane_distance);
    
    #pragma omp parallel for default(shared) \
                             private(obj_idx) \
                             shared(n_objects) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
		RenderableTriangleMesh& mesh = _objects[obj_idx];
		mesh.applyTransformation(_transformation_to_camera_system);
        mesh.computeNormals();
        mesh.clipNearPlaneAt(-_near_plane_distance);
        mesh.computeAABB();
    }

    _lights.applyTransformation(_transformation_to_camera_system);
        
    #pragma omp parallel for default(shared) \
                             private(x, y, obj_idx, face_idx, vertex_idx, n_triangles, x_min, x_max, y_min, y_max, pixel_center, alpha, beta, gamma, inverse_depths, vertices, normals, material, interpolated_inverse_depth, depth) \
                             shared(n_objects) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
        const RenderableTriangleMesh& mesh = _objects[obj_idx];

        n_triangles = mesh.getNumberOfFaces();

        for (face_idx = 0; face_idx < n_triangles; face_idx++)
        {
            Triangle2& projected_triangle = mesh.getProjectedFace(*this, face_idx);

            const AxisAlignedRectangle& aabb = projected_triangle.getAABR(_image_lower_corner, _image_upper_corner);

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
                    pixel_center.moveTo(static_cast<imp_float>(x) + 0.5f, static_cast<imp_float>(y) + 0.5f);

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
                                                                    material);

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
        }
    }

	if (gamma_encode)
		_image.gammaEncodeApprox(1.0f);

	_objects.clear();

    return *this;
}

Scene& Scene::generateGround(imp_float plane_height, imp_float plane_depth, const BlinnPhongMaterial& material)
{
    assert(plane_depth >= 0 && plane_depth < 1);

    const Ray& back_left_ray = _transformation_from_camera_system*_getEyeRay(Point2(0, plane_depth*_image_height));
    const Ray& back_right_ray = _transformation_from_camera_system*_getEyeRay(Point2(_image_width, plane_depth*_image_height));
    const Ray& front_center_ray = _transformation_from_camera_system*_getEyeRay(Point2(_image_width/2, 0));

    imp_float back_left_ray_distance = (plane_height - back_left_ray.origin.y)/back_left_ray.direction.y;
    imp_float back_right_ray_distance = (plane_height - back_right_ray.origin.y)/back_right_ray.direction.y;
    imp_float front_center_ray_distance = (plane_height - front_center_ray.origin.y)/front_center_ray.direction.y;

    assert(back_left_ray_distance >= 0 && back_right_ray_distance >= 0 && front_center_ray_distance >= 0);

    const Point& back_left = back_left_ray(back_left_ray_distance);
    const Point& back_right = back_right_ray(back_right_ray_distance);
    const Point& front_center = front_center_ray(front_center_ray_distance);

    const Vector& width_vector = back_right - back_left;
    const Vector& depth_vector = (back_left + width_vector/2) - front_center;
    const Point& front_left = front_center - width_vector/2;

    RenderableTriangleMesh& ground_mesh = RenderableTriangleMesh::sheet(front_left, width_vector, depth_vector);

    _objects.push_back(ground_mesh.withMaterial(material));

    return *this;
}

Geometry3D::Ray Scene::_getEyeRay(const Point2& pixel_center) const
{
    Vector unit_vector_from_camera_origin_to_pixel((pixel_center.x*_inverse_image_width - 0.5f)*_image_width_at_unit_distance_from_camera,
                                                   (pixel_center.y*_inverse_image_height - 0.5f)*_image_height_at_unit_distance_from_camera,
                                                   -1);

    unit_vector_from_camera_origin_to_pixel.normalize();

    return Ray((unit_vector_from_camera_origin_to_pixel*_near_plane_distance).toPoint(), unit_vector_from_camera_origin_to_pixel, _far_plane_distance);
}

Geometry2D::Point Scene::_getPerspectiveProjected(const arma::Col<imp_float>& vertex_vector) const
{
    imp_float normalization = -1/vertex_vector(2);

    return Point2(_image_width*(vertex_vector(0)*normalization*_inverse_image_width_at_unit_distance_from_camera + 0.5f),
                  _image_height*(vertex_vector(1)*normalization*_inverse_image_height_at_unit_distance_from_camera + 0.5f));
}

Radiance Scene::_getRadiance(const Point&  surface_point,
                             const Vector& surface_normal,
                             const Vector& scatter_direction,
                             const Material* material) const
{
    Radiance radiance = Color::black();

    imp_uint n_lights = _lights.getNumberOfLights();
    imp_uint n_samples;
    imp_uint n, s;
    Vector direction_to_source;
    imp_float distance_to_source;

    for (n = 0; n < n_lights; n++)
    {
        const Light* light = _lights.getLight(n);
        
        n_samples = light->getNumberOfSamples();

        for (s = 0; s < n_samples; s++)
        {
            const Vector4& source_point = light->getRandomPoint();

            if (source_point.w == 0)
            {
                direction_to_source = source_point.toVector();
                distance_to_source = IMP_FLOAT_INF;
            }
            else
            {
                direction_to_source = source_point.getXYZ() - surface_point;
                distance_to_source = direction_to_source.getLength();
                direction_to_source /= distance_to_source;
            }

            if (_evaluateVisibility(surface_point, direction_to_source, distance_to_source))
            {
                const Biradiance& biradiance = light->getBiradiance(source_point, surface_point);
                const Color& scattering_density = material->getScatteringDensity(surface_normal, direction_to_source, scatter_direction);
            
                radiance += surface_normal.dot(direction_to_source)*scattering_density*biradiance;
            }
        }

        radiance /= static_cast<float>(n_samples);
    }

    return radiance;
}

bool Scene::_evaluateVisibility(const Point& surface_point,
                                const Vector& direction_to_source,
                                imp_float distance_to_source) const
{   
    Ray shadow_ray(surface_point + direction_to_source*_ray_origin_offset, direction_to_source, IMP_FLOAT_INF);

    imp_uint n_objects = static_cast<imp_uint>(_objects.size());
    imp_uint obj_idx;
    std::vector<imp_uint>::const_iterator iter;

    for (obj_idx = 0; obj_idx < n_objects; obj_idx++)
    {
        const RenderableTriangleMesh& mesh = _objects[obj_idx];

        if (mesh.casts_shadows && mesh.evaluateRayIntersection(shadow_ray) < IMP_FLOAT_INF)
            return false;
    }

    return true;
}

} // Rendering3D
} // Impact
