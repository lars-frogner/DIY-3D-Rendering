#include "Renderer.hpp"
#include <cassert>
#include <omp.h>

namespace Impact {
namespace Rendering3D {

Renderer::Renderer(Image* new_image,
				   Camera* new_camera,
				   std::vector<Light*>* new_lights,
				   std::vector<Model*>* new_models)
    : _image(new_image),
	  _camera(new_camera),
	  _lights(new_lights),
	  _models(new_models) {}

void Renderer::computeViewData()
{
    _image_width = static_cast<imp_float>(_image->getWidth());
    _image_height = static_cast<imp_float>(_image->getHeight());
    
    _image_lower_corner.moveTo(0, 0);
    _image_upper_corner.moveTo(_image_width - 1, _image_height - 1);

    _inverse_image_width = 1/_image_width;
    _inverse_image_height = 1/_image_height;

    _image_width_at_unit_distance_from_camera = 2*tan(_camera->getFieldOfView()/2);
    _image_height_at_unit_distance_from_camera = _image_width_at_unit_distance_from_camera/_image->getAspectRatio();

    _inverse_image_width_at_unit_distance_from_camera = 1/_image_width_at_unit_distance_from_camera;
    _inverse_image_height_at_unit_distance_from_camera = 1/_image_height_at_unit_distance_from_camera;

	_camera_cframe = _camera->getCoordinateFrame();

    _transformation_to_camera_system = AffineTransformation::toCoordinateFrame(_camera_cframe);
    _transformation_from_camera_system = _transformation_to_camera_system.getInverse();

    _near_plane_distance = _camera->getNearPlaneDistance();
    _far_plane_distance = _camera->getFarPlaneDistance();

	_perspective_transformation = _camera->getWorldToParallelViewVolumeTransformation(_image->getAspectRatio());
	_windowing_transformation = _camera->getWindowingTransformation(_image->getWidth(), _image->getAspectRatio());

	_camera->getViewFrustumInCameraSystem(_image->getAspectRatio(),
										  _frustum_lower_plane,
										  _frustum_upper_plane,
										  _frustum_left_plane,
										  _frustum_right_plane);
}

void Renderer::storeLightWorldCoordinateFrames()
{
	imp_uint n_lights = static_cast<imp_uint>(_lights->size());

	_light_world_cframes.clear();
	_light_world_cframes.reserve(n_lights);

	for (imp_uint idx = 0; idx < n_lights; idx++)
	{
		_light_world_cframes.push_back(_lights->operator[](idx)->getCoordinateFrame());
	}
}

void Renderer::transformLightsToCameraSystem()
{
	for (std::vector<Light*>::iterator iter = _lights->begin(); iter != _lights->end(); iter++)
	{
		(*iter)->applyTransformation(_transformation_to_camera_system);
	}
}

void Renderer::transformLightsToWorldSystem()
{
	imp_uint n_lights = static_cast<imp_uint>(_lights->size());

	for (imp_uint idx = 0; idx < n_lights; idx++)
	{
		_lights->operator[](idx)->setCoordinateFrame(_light_world_cframes[idx]);
	}
}

void Renderer::createTransformedMeshCopies()
{
	assert(_mesh_copies.size() == 0);

	_mesh_copies.reserve(_models->size());

	for (std::vector<Model*>::const_iterator iter = _models->begin(); iter != _models->end(); iter++)
	{
		_mesh_copies.push_back((*iter)->getTransformedMesh());
	}
}

void Renderer::createTransformedMeshCopies(const AffineTransformation& additional_transformation)
{
	assert(_mesh_copies.size() == 0);

	_mesh_copies.reserve(_models->size());

	for (std::vector<Model*>::const_iterator iter = _models->begin(); iter != _models->end(); iter++)
	{
		_mesh_copies.push_back((*iter)->getTransformedMesh(additional_transformation));
	}
}

void Renderer::transformCameraLookRay(const AffineTransformation& transformation)
{
	_camera->transformLookRay(transformation);
	computeViewData();

	if (_lights_in_camera_system)
	{
		transformLightsToWorldSystem();
		transformLightsToCameraSystem();
	}
}

void Renderer::initialize()
{
	computeViewData();
	storeLightWorldCoordinateFrames();
}

void Renderer::renderDirect()
{
	int n_models = static_cast<int>(_models->size());
	int obj_idx;

	if (_lights_in_camera_system)
	{
		transformLightsToWorldSystem();
		_lights_in_camera_system = false;
	}

	createTransformedMeshCopies();
	
	_image->setBackgroundColor(bg_color);
    _image->initializeDepthBuffer(-1.0);
	
    #pragma omp parallel for default(shared) \
                             private(obj_idx) \
                             shared(n_models) \
                             schedule(dynamic) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_models; obj_idx++)
    {
		const Model* model = _models->operator[](obj_idx);
        TriangleMesh& mesh = _mesh_copies[obj_idx];

        if (n_splits > 0)
            mesh.splitFaces(n_splits);

		if (model->casts_shadows)
		{
			mesh.computeAABB();
			mesh.computeBoundingVolumeHierarchy();
		}
	}

    for (obj_idx = 0; obj_idx < n_models; obj_idx++)
    {
		const Model* model = _models->operator[](obj_idx);
        TriangleMesh& mesh = _mesh_copies[obj_idx];

        if (model->uses_direct_lighting)
            computeRadianceAtAllVertices(mesh, model->getMaterial());
	}

    #pragma omp parallel for default(shared) \
                             private(obj_idx) \
                             shared(n_models) \
                             schedule(dynamic) \
                             if (use_omp)
	for (obj_idx = 0; obj_idx < n_models; obj_idx++)
	{
		Model* model = _models->operator[](obj_idx);
		TriangleMesh& mesh = _mesh_copies[obj_idx];

		mesh.applyTransformation(_perspective_transformation);

		if (model->perform_clipping && mesh.allZAbove(0))
		{
			model->is_currently_visible = false;
			continue;
		}
		
		if (model->perform_clipping)
			mesh.clipNearPlane();

		mesh.homogenizeVertices();

		if (model->remove_hidden_faces)
			mesh.removeBackwardFacingFaces();

		if (model->perform_clipping)
		{
			mesh.computeAABB();

			if (mesh.isInsideParallelViewVolume())
			{
				mesh.clipNonNearPlanes();
			}
			else
			{
				model->is_currently_visible = false;
				continue;
			}
		}

		mesh.applyWindowingTransformation(_windowing_transformation);
	}

	for (obj_idx = 0; obj_idx < n_models; obj_idx++)
	{
		const Model* model = _models->operator[](obj_idx);
		const TriangleMesh& mesh = _mesh_copies[obj_idx];

		if (!model->is_currently_visible || !model->render_faces)
			continue;

        if (model->uses_direct_lighting)
            drawFaces(mesh);
        else
            drawFaces(mesh, model->getMaterial()->getBaseColor());
    }

	if (gamma_encode && !render_depth_map)
		_image->gammaEncodeApprox(1.0f);

	if (render_depth_map)
		_image->renderDepthMap(renormalize_depth_map);

	for (obj_idx = 0; obj_idx < n_models; obj_idx++)
	{
		Model* model = _models->operator[](obj_idx);
		const TriangleMesh& mesh = _mesh_copies[obj_idx];

		if (!model->is_currently_visible)
		{
			model->is_currently_visible = model->is_visible;
			continue;
		}
			
		if (draw_edges)
			drawEdges(mesh, edge_brightness);
	}

	_mesh_copies.clear();
}

void Renderer::rayTrace()
{
	int image_width = static_cast<int>(_image_width);
    int image_height = static_cast<int>(_image_height);
    int n_models = static_cast<int>(_models->size());
    int x, y, obj_idx;
    std::vector<imp_uint>::const_iterator iter;
    imp_float closest_distance;
    Point2 pixel_center;
    Radiance pixel_radiance;

	if (!_lights_in_camera_system)
	{
		transformLightsToCameraSystem();
		_lights_in_camera_system = true;
	}

	createTransformedMeshCopies(_transformation_to_camera_system);

	_image->setBackgroundColor(bg_color);
    
    #pragma omp parallel for default(shared) \
                             private(obj_idx) \
                             shared(n_models) \
                             schedule(dynamic) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_models; obj_idx++)
    {
		Model* model = _models->operator[](obj_idx);
        TriangleMesh& mesh = _mesh_copies[obj_idx];

		if (mesh.allZAbove(-_near_plane_distance))
		{
			model->is_currently_visible = false;

			if (model->casts_shadows)
			{
				mesh.computeAABB();
				mesh.computeBoundingVolumeHierarchy();
			}

			continue;
		}

		if (model->perform_clipping)
	        mesh.clipNearPlaneAt(-_near_plane_distance);

		mesh.computeAABB();

		if (mesh.isOutsideViewFrustum(_frustum_lower_plane,
									  _frustum_upper_plane,
								      _frustum_left_plane,
									  _frustum_right_plane,
									  _far_plane_distance))
		{
			model->is_currently_visible = false;

			if (model->casts_shadows)
			{
				mesh.computeAABB();
				mesh.computeBoundingVolumeHierarchy();
			}

			continue;
		}

		mesh.computeBoundingAreaHierarchy(_image_width,
									      _image_height,
									      _inverse_image_width_at_unit_distance_from_camera,
									      _inverse_image_height_at_unit_distance_from_camera);

		if (model->casts_shadows)
			mesh.computeBoundingVolumeHierarchy();
    }
    
    #pragma omp parallel for default(shared) \
                             private(x, y, pixel_center, obj_idx, iter, closest_distance, pixel_radiance) \
                             shared(image_height, image_width, n_models) \
                             schedule(dynamic) \
                             if (use_omp)
    for (y = 0; y < image_height; y++) {
        for (x = 0; x < image_width; x++)
        {
            pixel_center.moveTo(static_cast<imp_float>(x) + 0.5f, static_cast<imp_float>(y) + 0.5f);

            const Ray& eye_ray = getEyeRay(pixel_center);
                
            closest_distance = _far_plane_distance;

            for (obj_idx = 0; obj_idx < n_models; obj_idx++)
            {
				const Model* model = _models->operator[](obj_idx);

				if (model->is_currently_visible)
				{
					const TriangleMesh& mesh = _mesh_copies[obj_idx];
					const Material* material = model->getMaterial();

					const std::vector<imp_uint>& intersected_faces = mesh.getIntersectedFaceIndices(pixel_center);

					for (iter = intersected_faces.begin(); iter != intersected_faces.end(); iter++)
					{
						if (computeRadianceAtIntersectedPosition(mesh, material, eye_ray, *iter, pixel_radiance, closest_distance))
						{
							_image->setRadiance(x, y, (model->uses_direct_lighting)? pixel_radiance : model->getMaterial()->getBaseColor());
						}
					}
				}
            }
        }
    }
    
    for (obj_idx = 0; obj_idx < n_models; obj_idx++)
    {
		Model* model = _models->operator[](obj_idx);
		//const TriangleMesh& mesh = _mesh_copies[obj_idx];

		//if (model->render_edges)
			//drawEdges(mesh, edge_brightness);

		model->is_currently_visible = model->is_visible;
    }

	if (gamma_encode)
		_image->gammaEncodeApprox(1.0f);

	_mesh_copies.clear();
}

void Renderer::rasterize()
{
	int n_models = static_cast<int>(_models->size());
    int n_triangles;
    int x, y, obj_idx, face_idx, vertex_idx;
    int x_min, x_max, y_min, y_max;

    imp_float alpha, beta, gamma;
    Point2 pixel_center;
    Point vertices[3];
    Vector normals[3];
    
    imp_float inverse_depths[3];
    imp_float interpolated_inverse_depth;
    imp_float depth;

	if (!_lights_in_camera_system)
	{
		transformLightsToCameraSystem();
		_lights_in_camera_system = true;
	}
	
	createTransformedMeshCopies(_transformation_to_camera_system);

	_image->setBackgroundColor(bg_color);
    _image->initializeDepthBuffer(_far_plane_distance);
    
    #pragma omp parallel for default(shared) \
                             private(obj_idx) \
                             shared(n_models) \
							 schedule(dynamic) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_models; obj_idx++)
    {
		Model* model = _models->operator[](obj_idx);
        TriangleMesh& mesh = _mesh_copies[obj_idx];

		if (mesh.allZAbove(-_near_plane_distance))
		{
			model->is_currently_visible = false;

			if (model->casts_shadows)
			{
				mesh.computeAABB();
				mesh.computeBoundingVolumeHierarchy();
			}

			continue;
		}

		if (model->perform_clipping)
	        mesh.clipNearPlaneAt(-_near_plane_distance);

		mesh.computeAABB();

		if (mesh.isOutsideViewFrustum(_frustum_lower_plane,
									  _frustum_upper_plane,
								      _frustum_left_plane,
									  _frustum_right_plane,
									  _far_plane_distance))
		{
			model->is_currently_visible = false;

			if (model->casts_shadows)
			{
				mesh.computeAABB();
				mesh.computeBoundingVolumeHierarchy();
			}

			continue;
		}

		if (model->casts_shadows)
			mesh.computeBoundingVolumeHierarchy();
    }
        
    #pragma omp parallel for default(shared) \
                             private(x, y, obj_idx, face_idx, vertex_idx, n_triangles, x_min, x_max, y_min, y_max, pixel_center, alpha, beta, gamma, inverse_depths, vertices, normals, interpolated_inverse_depth, depth) \
                             shared(n_models) \
							 schedule(dynamic) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_models; obj_idx++)
    {
		Model* model = _models->operator[](obj_idx);
        const TriangleMesh& mesh = _mesh_copies[obj_idx];
		const Material* material = model->getMaterial();

		if (!model->is_currently_visible)
		{
			model->is_currently_visible = model->is_visible;
			continue;
		}

        n_triangles = mesh.getNumberOfFaces();

        for (face_idx = 0; face_idx < n_triangles; face_idx++)
        {
			if (!mesh.faceFacesOrigin(face_idx))
				continue;

            Triangle2& projected_triangle = mesh.getProjectedFace(face_idx,
																  _image_width,
																  _image_height,
																  _inverse_image_width_at_unit_distance_from_camera,
																  _inverse_image_height_at_unit_distance_from_camera);

            const AxisAlignedRectangle& aabb = projected_triangle.getAABR(_image_lower_corner, _image_upper_corner);

            x_min = static_cast<int>(aabb.lower_corner.x + 0.5f);
            x_max = static_cast<int>(aabb.upper_corner.x + 0.5f);
            y_min = static_cast<int>(aabb.lower_corner.y + 0.5f);
            y_max = static_cast<int>(aabb.upper_corner.y + 0.5f);

            projected_triangle.precomputeBarycentricQuantities();

            mesh.getFaceAttributes(face_idx, vertices, normals);

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

                        if (depth < _image->getDepth(x, y))
                        {
							if (model->uses_direct_lighting)
							{
								const Radiance& radiance = getScatteredRadiance(surface_point,
																				surface_normal,
																				displacement_to_camera/depth,
																				material);

								#pragma omp critical
								if (depth < _image->getDepth(x, y))
								{
									_image->setDepth(x, y, depth);
									_image->setRadiance(x, y, radiance);
								}
							}
							else
							{
								#pragma omp critical
								{
									_image->setDepth(x, y, depth);
									_image->setRadiance(x, y, model->getMaterial()->getBaseColor());
								}
							}
                        }
                    }
                }
            }
        }
    }

	if (gamma_encode && !render_depth_map)
		_image->gammaEncodeApprox(1.0f);

	if (render_depth_map)
		_image->renderDepthMap(renormalize_depth_map);

	_mesh_copies.clear();
}

Geometry3D::Ray Renderer::getEyeRay(const Point2& pixel_center) const
{
    Vector unit_vector_from_camera_origin_to_pixel((pixel_center.x*_inverse_image_width - 0.5f)*_image_width_at_unit_distance_from_camera,
                                                   (pixel_center.y*_inverse_image_height - 0.5f)*_image_height_at_unit_distance_from_camera,
                                                   -1);

    unit_vector_from_camera_origin_to_pixel.normalize();

    return Ray((unit_vector_from_camera_origin_to_pixel*_near_plane_distance).toPoint(), unit_vector_from_camera_origin_to_pixel, _far_plane_distance);
}

Radiance Renderer::getScatteredRadiance(const Point&  surface_point,
										const Vector& surface_normal,
										const Vector& scatter_direction,
										const Material* material) const
{
    Radiance radiance = Color::black();

    imp_uint n_lights = static_cast<imp_uint>(_lights->size());
    imp_uint n_samples;
    imp_uint s;
    Vector direction_to_source;
    imp_float distance_to_source;

    for (std::vector<Light*>::const_iterator iter = _lights->begin(); iter != _lights->end(); iter++)
    {
        const Light* light = *iter;
        
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

            if (!(light->creates_shadows) || evaluateSourceVisibility(surface_point, direction_to_source, distance_to_source))
            {
                const Biradiance& biradiance = light->getBiradiance(source_point, surface_point, distance_to_source);
                const Color& scattering_density = material->getScatteringDensity(surface_normal, direction_to_source, scatter_direction);
            
                radiance += surface_normal.dot(direction_to_source)*scattering_density*biradiance;
            }
        }

        radiance /= static_cast<imp_float>(n_samples);

		radiance += light->ambient_radiance;
    }

    return radiance;
}

bool Renderer::evaluateSourceVisibility(const Point& surface_point,
										const Vector& direction_to_source,
										imp_float distance_to_source) const
{   
	imp_uint n_models = static_cast<imp_uint>(_models->size());
	imp_uint obj_idx;

    Ray shadow_ray(surface_point + direction_to_source*_ray_origin_offset, direction_to_source, distance_to_source);

    for (obj_idx = 0; obj_idx < n_models; obj_idx++)
    {
		const Model* model = _models->operator[](obj_idx);
        const TriangleMesh& mesh = _mesh_copies[obj_idx];

        if (model->casts_shadows && mesh.evaluateRayIntersection(shadow_ray) < distance_to_source)
            return false;
    }

    return true;
}

void Renderer::computeRadianceAtAllVertices(TriangleMesh& mesh, const Material* material)
{
    assert(mesh.isHomogenized());
    assert(mesh.hasNormals());

    int n_vertices = static_cast<int>(mesh.getNumberOfVertices());
    int idx;
	imp_float vertex_camera_distance;
	
	mesh.initializeVertexData3();

    #pragma omp parallel for default(shared) \
                             private(idx, vertex_camera_distance) \
                             shared(mesh, material, n_vertices) \
							 schedule(static) \
                             if (use_omp)
    for (idx = 0; idx < n_vertices; idx++)
    {
		const Point& vertex_point = mesh.getVertex(idx);
		const Vector& normal_vector = mesh.getVertexNormal(idx);

        Vector& scatter_direction = _camera_cframe.origin - vertex_point;
		vertex_camera_distance = scatter_direction.getLength();

		if (vertex_camera_distance > 0)
		{
			scatter_direction /= vertex_camera_distance;

			const Radiance& radiance = getScatteredRadiance(vertex_point,
															normal_vector,
															scatter_direction,
															material).clamp();

			mesh.setVertexData3(idx, radiance.r, radiance.g, radiance.b);
		}
    }
}

bool Renderer::computeRadianceAtIntersectedPosition(const TriangleMesh& mesh,
													const Material* material,
													const Ray& ray,
													imp_uint face_idx,
													Radiance& pixel_radiance,
													imp_float& closest_distance) const
{
    assert(mesh.hasNormals());

    imp_float alpha, beta, gamma;
    imp_float distance = mesh.evaluateRayFaceIntersection(ray, face_idx, alpha, beta, gamma);

    if (distance >= closest_distance)
        return false;

    closest_distance = distance;

    const Point& intersection_point = ray(distance);

	Vector face_normals[3];
	mesh.getFaceNormals(face_idx, face_normals);

	Vector interpolated_normal(face_normals[0]*alpha + face_normals[1]*beta + face_normals[2]*gamma);
    interpolated_normal.normalize();

    pixel_radiance = getScatteredRadiance(intersection_point,
										  interpolated_normal,
										  -ray.direction,
										  material);

    return true;
}

void Renderer::drawFaces(const TriangleMesh& mesh) const
{
    assert(mesh.isHomogenized());

    int n_faces = static_cast<int>(mesh.getNumberOfFaces());
    int face_idx;
	Point face_vertices[3];
	imp_float vertex_color_A[3];
	imp_float vertex_color_B[3];
	imp_float vertex_color_C[3];
    
    #pragma omp parallel for default(shared) \
                             private(face_idx, face_vertices, vertex_color_A, vertex_color_B, vertex_color_C) \
                             shared(mesh, n_faces) \
							 schedule(dynamic) \
                             if (use_omp)
    for (face_idx = 0; face_idx < n_faces; face_idx++)
    {
		mesh.getFaceVertices(face_idx, face_vertices);
		mesh.getFaceVertexData3(face_idx, vertex_color_A, vertex_color_B, vertex_color_C);

        _image->drawTriangle(face_vertices[0], face_vertices[1], face_vertices[2],
		 				     Color(vertex_color_A[0], vertex_color_A[1], vertex_color_A[2]),
						     Color(vertex_color_B[0], vertex_color_B[1], vertex_color_B[2]),
						     Color(vertex_color_C[0], vertex_color_C[1], vertex_color_C[2]));
    }
}

void Renderer::drawFaces(const TriangleMesh& mesh, Color color) const
{
    assert(mesh.isHomogenized());

    imp_uint n_faces = mesh.getNumberOfFaces();
	Point face_vertices[3];

    for (imp_uint face_idx = 0; face_idx < n_faces; face_idx++)
    {
		mesh.getFaceVertices(face_idx, face_vertices);

        _image->drawTriangle(face_vertices[0], face_vertices[1], face_vertices[2],
						    color, color, color);
    }
}

void Renderer::drawEdges(const TriangleMesh& mesh, float luminance) const
{
    assert(mesh.isHomogenized());

    imp_uint n_faces = mesh.getNumberOfFaces();
	Point face_vertices[3];

    for (imp_uint face_idx = 0; face_idx < n_faces; face_idx++)
    {
		mesh.getFaceVertices(face_idx, face_vertices);

        _image->drawLine(face_vertices[0].x, face_vertices[0].y,
                        face_vertices[1].x, face_vertices[1].y, luminance);

        _image->drawLine(face_vertices[1].x, face_vertices[1].y,
                        face_vertices[2].x, face_vertices[2].y, luminance);
        
        _image->drawLine(face_vertices[2].x, face_vertices[2].y,
                        face_vertices[0].x, face_vertices[0].y, luminance);
    }
}

void Renderer::saveImage(const std::string& filename)
{
	_image->saveAsPPM(filename);
}

const Image& Renderer::getImage() const
{
	return *_image;
}

const Geometry3D::Camera& Renderer::getCamera() const
{
	return *_camera;
}

void Renderer::toggleModelShadows()
{
	std::vector<Model*>::iterator iter = _models->begin();

	for (; iter != _models->end(); iter++)
	{
		if ((*iter)->shadows_toggable)
			(*iter)->casts_shadows = !((*iter)->casts_shadows);
	}
}

} // Rendering3D
} // Impact
