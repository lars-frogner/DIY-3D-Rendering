#include "Renderer.hpp"
#include "UniformSubpixelSampler.hpp"
#include "StratifiedSubpixelSampler.hpp"
#include "math_util.hpp"
#include <cassert>
#include <list>
#include <forward_list>
#include <omp.h>

namespace Impact {
namespace Rendering3D {

Renderer::Renderer(Image* new_image,
				   Camera* new_camera,
				   std::vector<OmnidirectionalLight*>* new_point_lights,
				   std::vector<DirectionalLight*>* new_directional_lights,
				   std::vector<AreaLight*>* new_area_lights,
				   std::vector<Model*>* new_models)
    : _image(new_image),
	  _camera(new_camera),
	  _point_lights(new_point_lights),
	  _directional_lights(new_directional_lights),
	  _area_lights(new_area_lights),
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
	imp_uint idx;
	imp_uint n_point_lights = static_cast<imp_uint>(_point_lights->size());
	imp_uint n_directional_lights = static_cast<imp_uint>(_directional_lights->size());
	imp_uint n_area_lights = static_cast<imp_uint>(_area_lights->size());

	_light_world_cframes.clear();
	_light_world_cframes.reserve(n_point_lights + n_directional_lights + n_area_lights);

	for (idx = 0; idx < n_point_lights; idx++)
	{
		_light_world_cframes.push_back(_point_lights->operator[](idx)->getCoordinateFrame());
	}

	for (idx = 0; idx < n_directional_lights; idx++)
	{
		_light_world_cframes.push_back(_directional_lights->operator[](idx)->getCoordinateFrame());
	}

	for (idx = 0; idx < n_area_lights; idx++)
	{
		_light_world_cframes.push_back(_area_lights->operator[](idx)->getCoordinateFrame());
	}
}

void Renderer::transformLightsToCameraSystem()
{
	for (std::vector<OmnidirectionalLight*>::iterator iter = _point_lights->begin(); iter != _point_lights->end(); iter++)
	{
		(*iter)->applyTransformation(_transformation_to_camera_system);
	}

	for (std::vector<DirectionalLight*>::iterator iter = _directional_lights->begin(); iter != _directional_lights->end(); iter++)
	{
		(*iter)->applyTransformation(_transformation_to_camera_system);
	}

	for (std::vector<AreaLight*>::iterator iter = _area_lights->begin(); iter != _area_lights->end(); iter++)
	{
		(*iter)->applyTransformation(_transformation_to_camera_system);
	}
}

void Renderer::transformLightsToWorldSystem()
{
	imp_uint idx;
	imp_uint n_point_lights = static_cast<imp_uint>(_point_lights->size());
	imp_uint n_directional_lights = static_cast<imp_uint>(_directional_lights->size());
	imp_uint n_area_lights = static_cast<imp_uint>(_area_lights->size());

	for (idx = 0; idx < n_point_lights; idx++)
	{
		_point_lights->operator[](idx)->setCoordinateFrame(_light_world_cframes[idx]);
	}

	for (idx = 0; idx < n_directional_lights; idx++)
	{
		_directional_lights->operator[](idx)->setCoordinateFrame(_light_world_cframes[n_point_lights + idx]);
	}

	for (idx = 0; idx < n_area_lights; idx++)
	{
		_area_lights->operator[](idx)->setCoordinateFrame(_light_world_cframes[(n_point_lights + n_directional_lights) + idx]);
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

void Renderer::buildMeshBVH()
{
	int idx;
	int n_meshes = static_cast<int>(_mesh_copies.size());

    std::vector<AABBContainer> objects(n_meshes);
	AxisAlignedBox aabb(Point::max(), Point::min());
	
    /*#pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(n_meshes, objects, aabb) \
                             schedule(dynamic) \
                             if (use_omp)*/
	for (idx = 0; idx < n_meshes; idx++)
	{
		_mesh_copies[idx].computeAABB();
		_mesh_copies[idx].computeBoundingVolumeHierarchy();

        objects[idx].aabb = _mesh_copies[idx].getAABB();
        objects[idx].centroid = _mesh_copies[idx].getCentroid();
        objects[idx].id = idx;

		#pragma omp critical
		aabb.merge(objects[idx].aabb);
	}

    _mesh_BVH = BoundingVolumeHierarchy(aabb, objects);
	
    objects.clear();
}

void Renderer::buildMeshBVHForShadowsOnly()
{
	int idx;
	int n_meshes = static_cast<int>(_mesh_copies.size());

    std::vector<AABBContainer> objects;
	AxisAlignedBox aabb(Point::max(), Point::min());

	objects.reserve(n_meshes);
	
    #pragma omp parallel for default(shared) \
                             private(idx) \
                             shared(n_meshes, objects, aabb) \
                             schedule(dynamic) \
                             if (use_omp)
	for (idx = 0; idx < n_meshes; idx++)
	{
		if (_models->operator[](idx)->casts_shadows && _mesh_copies[idx].getNumberOfFaces() > 0)
		{
			_mesh_copies[idx].computeAABB();
			_mesh_copies[idx].computeBoundingVolumeHierarchy();

			#pragma omp critical
			{
				objects.emplace_back(_mesh_copies[idx].getAABB(),
									 _mesh_copies[idx].getCentroid(),
									 idx);

				aabb.merge(objects.back().aabb);
			}
		}
	}

	if (objects.size() > 0)
	    _mesh_BVH = BoundingVolumeHierarchy(aabb, objects);
	else
	    _mesh_BVH = BoundingVolumeHierarchy();
	
    objects.clear();
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
	SurfaceElement surface_element;

	if (_lights_in_camera_system)
	{
		transformLightsToWorldSystem();
		_lights_in_camera_system = false;
	}

	createTransformedMeshCopies();
	
	_image->setBackgroundColor(bg_color);
    _image->initializeDepthBuffer(-1.0);

	buildMeshBVHForShadowsOnly();

    for (obj_idx = 0; obj_idx < n_models; obj_idx++)
    {
		const Model* model = _models->operator[](obj_idx);
        TriangleMesh& mesh = _mesh_copies[obj_idx];

		surface_element.model = model;

        if (model->uses_direct_lighting)
            computeRadianceAtAllVertices(mesh, surface_element);
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

		if (mesh.getNumberOfFaces() == 0)
		{
			model->is_currently_visible = false;
			continue;
		}

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
			drawEdgesInImageSpace(mesh, edge_brightness);
	}

	_mesh_copies.clear();
}

void Renderer::rayTrace()
{
    int x, y;
	int image_width = static_cast<int>(_image_width);
    int image_height = static_cast<int>(_image_height);
    imp_float closest_distance;
    Point2 pixel_center;
    Radiance pixel_radiance;
	SurfaceElement surface_element;

	if (!_lights_in_camera_system)
	{
		transformLightsToCameraSystem();
		_lights_in_camera_system = true;
	}

	createTransformedMeshCopies(_transformation_to_camera_system);

    buildMeshBVH();

	printPickInfo();

	_image->setBackgroundColor(bg_color);

    #pragma omp parallel for default(shared) \
							 private(x, y, pixel_center, closest_distance, surface_element) \
							 shared(image_height, image_width) \
							 schedule(dynamic) \
							 if (use_omp)
	for (y = 0; y < image_height; y++) {
		for (x = 0; x < image_width; x++)
		{
			pixel_center.moveTo(static_cast<imp_float>(x) + 0.5f, static_cast<imp_float>(y) + 0.5f);

			const Ray& eye_ray = getEyeRay(pixel_center);
                
			closest_distance = _far_plane_distance;

			if (findIntersectedSurface(eye_ray, closest_distance, surface_element))
			{
				_image->setRadiance(x, y, getDirectlyScatteredRadianceFromLights(surface_element, -eye_ray.direction));
			}
		}
	}

	if (gamma_encode)
		_image->gammaEncodeApprox(1.0f);

	if (draw_edges)
	{
		for (imp_uint obj_idx = 0; obj_idx < static_cast<imp_uint>(_models->size()); obj_idx++)
		{
			Model* model = _models->operator[](obj_idx);
			const TriangleMesh& mesh = _mesh_copies[obj_idx];
			
			drawEdgesInCameraSpace(mesh, edge_brightness);
		}
	}

	_mesh_copies.clear();
}

void Renderer::rasterize()
{
	int n_models = static_cast<int>(_models->size());
    int n_triangles;
    int x, y, obj_idx, face_idx;
    int x_min, x_max, y_min, y_max;

    imp_float alpha, beta, gamma;
    Point2 pixel_center;
    Point vertices[3];
    Vector normals[3];
    Point2 texture_coordinates[3];
	Vector tangents[3], bitangents[3];
    
    imp_float inverse_depths[3];
    imp_float interpolated_inverse_depth;
    imp_float depth;

	SurfaceElement surface_element;

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
			continue;
		}

		if (model->perform_clipping)
	        mesh.clipNearPlaneAt(-_near_plane_distance);

		mesh.computeAABB();

		//if (mesh.isOutsideViewFrustum(_frustum_lower_plane,
		//							  _frustum_upper_plane,
		//						      _frustum_left_plane,
		//							  _frustum_right_plane,
		//							  _far_plane_distance))
		//{
		//	model->is_currently_visible = false;
		//	continue;
		//}
    }

	buildMeshBVHForShadowsOnly();
        
    #pragma omp parallel for default(shared) \
                             private(x, y, obj_idx, face_idx, n_triangles, x_min, x_max, y_min, y_max, pixel_center, alpha, beta, gamma, inverse_depths, vertices, normals, texture_coordinates, tangents, bitangents, interpolated_inverse_depth, depth, surface_element) \
                             shared(n_models) \
							 schedule(dynamic) \
                             if (use_omp)
    for (obj_idx = 0; obj_idx < n_models; obj_idx++)
    {
		Model* model = _models->operator[](obj_idx);

		if (!model->is_currently_visible)
		{
			model->is_currently_visible = model->is_visible;
			continue;
		}

		surface_element.model = model;

        const TriangleMesh& mesh = _mesh_copies[obj_idx];

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

			mesh.getFaceVertices(face_idx, vertices);
			mesh.getVertexNormalsForFace(face_idx, normals);

			if (model->hasTexture())
			{
				mesh.getTextureCoordinates(face_idx, texture_coordinates);

				if (model->hasNormalMap())
				{
					mesh.getVertexTangentsForFace(face_idx, tangents, bitangents);
				}
			}

			surface_element.geometric.normal = mesh.getFaceNormal(face_idx);

			inverse_depths[0] = -1.0f/vertices[0].z;
			inverse_depths[1] = -1.0f/vertices[1].z;
			inverse_depths[2] = -1.0f/vertices[2].z;

			vertices[0] *= inverse_depths[0];
			vertices[1] *= inverse_depths[1];
			vertices[2] *= inverse_depths[2];

			normals[0] *= inverse_depths[0];
			normals[1] *= inverse_depths[1];
			normals[2] *= inverse_depths[2];
			
			if (model->hasTexture())
			{
				texture_coordinates[0] *= inverse_depths[0];
				texture_coordinates[1] *= inverse_depths[1];
				texture_coordinates[2] *= inverse_depths[2];
				
				if (model->hasNormalMap())
				{
					tangents[0] *= inverse_depths[0];
					tangents[1] *= inverse_depths[1];
					tangents[2] *= inverse_depths[2];
					
					bitangents[0] *= inverse_depths[0];
					bitangents[1] *= inverse_depths[1];
					bitangents[2] *= inverse_depths[2];
				}
			}

			for (y = y_min; y < y_max; y++) {
				for (x = x_min; x < x_max; x++)
				{
					pixel_center.moveTo(static_cast<imp_float>(x) + 0.5f, static_cast<imp_float>(y) + 0.5f);

					projected_triangle.getBarycentricCoordinates(pixel_center, alpha, beta, gamma);

					if (alpha > 0 && beta > 0 && gamma > 0)
					{
						interpolated_inverse_depth = alpha*inverse_depths[0] + beta*inverse_depths[1] + gamma*inverse_depths[2];

						surface_element.geometric.position = (vertices[0] + (vertices[1] - vertices[0])*beta + (vertices[2] - vertices[0])*gamma)/interpolated_inverse_depth;
						surface_element.shading.position = surface_element.geometric.position;

						surface_element.shading.normal = ((normals[0]*alpha + normals[1]*beta + normals[2]*gamma)/interpolated_inverse_depth).getNormalized();

						surface_element.shading.color = Color::white();

						if (model->hasTexture())
						{
							surface_element.shading.texture_coordinate = (texture_coordinates[0] + (texture_coordinates[1] - texture_coordinates[0])*beta + (texture_coordinates[2] - texture_coordinates[0])*gamma)/interpolated_inverse_depth;
							
							if (model->hasColorTexture())
							{
								surface_element.computeTextureColor();
							}

							/*if (model->hasDisplacementMap())
							{
								surface_element.computeDisplacementMappedPosition();
							}*/

							if (surface_element.evaluateNormalMapping())
							{
								surface_element.shading.tangent = ((tangents[0]*alpha + tangents[1]*beta + tangents[2]*gamma)/interpolated_inverse_depth).getNormalized();
								surface_element.shading.bitangent = ((bitangents[0]*alpha + bitangents[1]*beta + bitangents[2]*gamma)/interpolated_inverse_depth).getNormalized();

								surface_element.computeNormalMappedNormal();
							}
						}

						const Vector& displacement_to_camera = -(surface_element.shading.position.toVector());

						depth = displacement_to_camera.getLength();

						if (depth < _image->getDepth(x, y))
						{
							if (model->uses_direct_lighting)
							{
								const Radiance& radiance = getDirectlyScatteredRadianceFromLights(surface_element,
																								  displacement_to_camera/depth);

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

	if (draw_edges)
	{
		for (obj_idx = 0; obj_idx < n_models; obj_idx++)
		{
			Model* model = _models->operator[](obj_idx);
			const TriangleMesh& mesh = _mesh_copies[obj_idx];
			
			drawEdgesInCameraSpace(mesh, edge_brightness);
		}
	}

	_mesh_copies.clear();
}

void Renderer::pathTrace(imp_uint n_samples)
{
    int x, y;
	int image_width = static_cast<int>(_image_width);
    int image_height = static_cast<int>(_image_height);
	imp_uint i, j;
    Point2 pixel_center;
	Radiance pixel_radiance;
	imp_float sample_normalizatiion = 1/static_cast<imp_float>(n_samples);
	imp_uint samplegrid_resolution = static_cast<imp_uint>(floor(sqrt(static_cast<imp_float>(n_samples))));
	imp_float samplegrid_cell_size = 1.0f/samplegrid_resolution;
	imp_uint remaining_samples = n_samples - samplegrid_resolution*samplegrid_resolution;
	Medium ray_medium;

	if (!_lights_in_camera_system)
	{
		transformLightsToCameraSystem();
		_lights_in_camera_system = true;
	}

	createTransformedMeshCopies(_transformation_to_camera_system);

    buildMeshBVH();

	printPickInfo();

	_image->setBackgroundColor(bg_color);

    #pragma omp parallel for default(shared) \
							 private(x, y, i, j, pixel_center, pixel_radiance, ray_medium) \
							 shared(image_height, image_width, n_samples, sample_normalizatiion, samplegrid_resolution, samplegrid_cell_size, remaining_samples) \
							 schedule(dynamic) \
							 if (use_omp)
	for (y = 0; y < image_height; y++) {
		for (x = 0; x < image_width; x++)
		{
			pixel_radiance = Radiance::black();
			
			for (i = 0; i < samplegrid_resolution; i++) {
				for (j = 0; j < samplegrid_resolution; j++)
				{
					pixel_center.moveTo(static_cast<imp_float>(x) + (i + math_util::random())*samplegrid_cell_size,
										static_cast<imp_float>(y) + (j + math_util::random())*samplegrid_cell_size);
					pixel_radiance += pathTrace(getEyeRay(pixel_center), ray_medium, 0);
				}
			}
			
			for (i = 0; i < remaining_samples; i++)
			{
				pixel_center.moveTo(static_cast<imp_float>(x) + math_util::random(),
									static_cast<imp_float>(y) + math_util::random());
				pixel_radiance += pathTrace(getEyeRay(pixel_center), ray_medium, 0);
			}

			_image->setRadiance(x, y, pixel_radiance*sample_normalizatiion);
		}
	}

	if (gamma_encode)
		_image->gammaEncodeApprox(1.0f);

	if (draw_edges)
	{
		for (imp_uint obj_idx = 0; obj_idx < static_cast<imp_uint>(_models->size()); obj_idx++)
		{
			Model* model = _models->operator[](obj_idx);
			const TriangleMesh& mesh = _mesh_copies[obj_idx];
			
			drawEdgesInCameraSpace(mesh, edge_brightness);
		}
	}

	_mesh_copies.clear();
}

Renderer::ImageRectangle::ImageRectangle(imp_int new_x_start, imp_int new_x_end,
										 imp_int new_y_start, imp_int new_y_end)
	: start{new_x_start, new_y_start},
	  end{new_x_end, new_y_end},
	  extent{new_x_end - new_x_start, new_y_end - new_y_start},
	  has_smallest_extent{false, false},
	  n_samples_accumulated(0) {}

Renderer::ImageRectangle::ImageRectangle(const ImageRectangle& other)
	: start{other.start[0], other.start[1]},
	  end{other.end[0], other.end[1]},
	  extent{other.extent[0], other.extent[1]},
	  has_smallest_extent{other.has_smallest_extent[0], other.has_smallest_extent[1]},
	  n_samples_accumulated(other.n_samples_accumulated) {}

Renderer::ImageRectangle& Renderer::ImageRectangle::operator=(const ImageRectangle& other)
{
	start[0] = other.start[0];
	start[1] = other.start[1];
	end[0] = other.end[0];
	end[1] = other.end[1];
	extent[0] = other.extent[0];
	extent[1] = other.extent[1];
	has_smallest_extent[0] = other.has_smallest_extent[0];
	has_smallest_extent[1] = other.has_smallest_extent[1];
	n_samples_accumulated = other.n_samples_accumulated;
	return *this;
}

Renderer::ImageRectangle::ImageRectangle(const ImageRectangle& other,
										 imp_uint new_start_dimension, imp_int new_start)
	: start{other.start[0], other.start[1]},
	  end{other.end[0], other.end[1]},
	  has_smallest_extent{false, false},
	  n_samples_accumulated(other.n_samples_accumulated)
{
	start[new_start_dimension] = new_start;

	extent[0] = end[0] - start[0];
	extent[1] = end[1] - start[1];
}

void Renderer::ImageRectangle::setDimensionOfLargestExtent(imp_uint& dimension, imp_uint& other_dimension) const
{
	if (extent[1] > extent[0])
	{
		dimension = 1;
		other_dimension = 0;
	}
	else
	{
		dimension = 0;
		other_dimension = 1;
	}
}

void Renderer::ImageRectangle::setEnd(imp_uint dimension, imp_int new_end)
{
	end[dimension] = new_end;
	extent[dimension] = new_end - start[dimension];
}

void Renderer::pathTraceAdaptive(imp_float tolerance)
{
	if (!_lights_in_camera_system)
	{
		transformLightsToCameraSystem();
		_lights_in_camera_system = true;
	}

	createTransformedMeshCopies(_transformation_to_camera_system);

    buildMeshBVH();

	printPickInfo();

	_image->setBackgroundColor(bg_color);

	const imp_uint n_samples_between_checks = 20;
	assert(n_samples_between_checks % 2 == 0);

	const imp_float termination_threshold = tolerance;
	const imp_float splitting_threshold = 256.0f*termination_threshold;

	const imp_int smallest_block_extent = 4;

	// Create secondary image for accumulating half of the samples
	// used in the primary image
	Image* secondary_image = new Image(_image->getWidth(), _image->getHeight());
	secondary_image->setBackgroundColor(bg_color);

	// Create array for holding pixel errors accumulated along the
	// horizontal or vertical direction of a block.
	imp_float* accumulated_pixel_errors = new imp_float[(_image->getWidth() > _image->getHeight())? _image->getWidth() : _image->getHeight()];
	imp_float accumulated_total_pixel_error, total_pixel_error;
	imp_float block_error;
	imp_uint n_invalid_pixels;
	imp_float sample_norm_primary, sample_norm_secondary;

	// Create linked list for storing blocks
	std::list<ImageRectangle> blocks, new_blocks;
	std::list<ImageRectangle>::iterator block;

	// Start with one block covering the entire image
	blocks.emplace_back(0, _image->getWidth(), 0, _image->getHeight());

	imp_uint split_dimension, other_dimension;
	
	//private(i, j, n, idx, eye_ray_origin, ray_medium, pixel_error) 
	//shared(n_samples_between_checks, termination_threshold, splitting_threshold, smallest_block_extent, secondary_image, accumulated_pixel_errors, accumulated_total_pixel_error, total_pixel_error, block_error, n_invalid_pixels, sample_norm_primary, sample_norm_secondary, blocks, new_blocks, block, split_dimension, other_dimension) \

	//#pragma omp parallel default(shared) \
	//if (use_omp)s
	//{
		
    imp_int i, j, n, idx[2];
	Medium ray_medium;
	imp_float pixel_error;
	//bool splitted;
	
	UniformSubpixelSampler sampler;
	//StratifiedSubpixelSampler sampler;

	while (!blocks.empty()) // Keep sampling as long as there are active blocks
	{
		//#pragma omp barrier
		//#pragma omp master
		//block = blocks.begin();
		//#pragma omp barrier

		// Iterate through the list of blocks
		for (block = blocks.begin(); block != blocks.end();)
		{
			//if (block->valid == true)
			//{
				
			//#pragma omp barrier
			//#pragma omp master
			//std::cout << "sampling" << std::endl;
			
			//#pragma omp barrier

			/*if (block->start[0] < 0)
			{
				std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << block->start[0] << std::endl;
				throw;
			}
			if (block->start[1] < 0)
			{
				std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << block->start[1] << std::endl;
				throw;
			}
			if (block->end[0] > _image->getWidth())
			{
				std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << block->end[0] << std::endl;
				throw;
			}
			if (block->end[1] > _image->getHeight())
			{
				std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << block->end[1] << std::endl;
				throw;
			}
			if (block->end[0] <= block->start[0])
			{
				std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << block->start[0] << " " << block->end[0] << std::endl;
				throw;
			}
			if (block->end[1] <= block->start[1])
			{
				std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << block->start[1] << " " << block->end[1] << std::endl;
				throw;
			}
			if (block->extent[0] != block->end[0] - block->start[0])
			{
				std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << block->extent[0] << std::endl;
				throw;
			}
			if (block->extent[1] != block->end[1] - block->start[1])
			{
				std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << block->extent[1] << std::endl;
				throw;
			}*/

			sampler.initializeSampling(n_samples_between_checks);
			
			//#pragma omp barrier
			// Accumulate samples for each pixel of the block
			#pragma omp parallel for default(shared) private(i, j, n, sampler, ray_medium) schedule(dynamic)
			for (j = block->start[1]; j < block->end[1]; j++) {
				for (i = block->start[0]; i < block->end[0]; i++)
				{
					sampler.setPixel(i, j);

					for (n = 0; n < n_samples_between_checks; n += 2)
					{
						_image->accumulateRadiance(i, j, pathTrace(getEyeRay(sampler.samplePixelPoint()), ray_medium, 0));

						const Radiance& pixel_radiance = pathTrace(getEyeRay(sampler.samplePixelPoint()), ray_medium, 0);

						_image->accumulateRadiance(i, j, pixel_radiance);

						// Add half the samples to a separate image (needed for evaluating image variance)
						secondary_image->accumulateRadiance(i, j, pixel_radiance);
					}
				}
			}
			
			//#pragma omp barrier
			//#pragma omp master
			//{
			// Update current number of samples accumulated for the block
			block->n_samples_accumulated += n_samples_between_checks;

			// Compute normalization factors for both images (primary and the one with only half the samples)
			sample_norm_primary = 1.0f/block->n_samples_accumulated;
			sample_norm_secondary = 2*sample_norm_primary;

			// Find largest dimension of the block (for splitting)
			block->setDimensionOfLargestExtent(split_dimension, other_dimension);
			
			// Also keep track of the number of pixels that are excluded from variance estimation
			n_invalid_pixels = 0;

			total_pixel_error = 0.0f;

			//std::cout << "estimating variance" << std::endl;
			//}
			//#pragma omp barrier

			//#pragma omp for schedule(static)
			#pragma omp parallel for default(shared) private(i, j, n, idx, pixel_error) reduction(+:total_pixel_error) schedule(static)
			for (n = block->start[split_dimension]; n < block->end[split_dimension]; n++)
			{
				idx[split_dimension] = n;

				pixel_error = 0.0f;

				for (idx[other_dimension] = block->start[other_dimension]; idx[other_dimension] < block->end[other_dimension]; idx[other_dimension]++)
				{
					i = idx[0];
					j = idx[1];

					if (_image->getRadiance(i, j).nonZero())
					{
						const Color& difference = _image->getApproxGammaEncodedColor(i, j, sample_norm_primary) -
												  secondary_image->getApproxGammaEncodedColor(i, j, sample_norm_secondary);

						pixel_error += (difference*difference).getTotal();
					}
					else
					{
						#pragma omp atomic
						n_invalid_pixels++;
					}
				}

				// Accumulate total pixel error
				total_pixel_error += pixel_error;

				// Store pixel errors accumulated over non-split dimension
				accumulated_pixel_errors[n] = pixel_error;
			}

			//#pragma omp barrier
			//#pragma omp master
			//{
			block_error = sqrt(total_pixel_error/(block->extent[0]*block->extent[1] - n_invalid_pixels));
			std::cout << "block_error = " << block_error << std::endl;
			//}
			
			//#pragma omp barrier
			if (block_error < splitting_threshold) // Is the error small for the current block?
			{
				if (block_error < termination_threshold) // Is the error small enough to stop sampling in this block?
				{
					// Normalize samples
					
					//#pragma omp barrier
					//#pragma omp master
					std::cout << "block of size (" << block->extent[0] << ", " << block->extent[1] << ") terminated with n_samples = " << block->n_samples_accumulated << std::endl;
					//#pragma omp barrier
						
					//#pragma omp barrier
					//#pragma omp for schedule(static)
					#pragma omp parallel for default(shared) private(i, j) schedule(static)
					for (j = block->start[1]; j < block->end[1]; j++) {
						for (i = block->start[0]; i < block->end[0]; i++)
						{
							_image->normalizeAccumulatedRadiance(i, j, sample_norm_primary);

							secondary_image->setRadiance(i, j, Radiance::grey(static_cast<imp_float>(block->n_samples_accumulated)));
						}
					}
					//#pragma omp barrier

					// Remove the block from the list
					//#pragma omp barrier 
					//#pragma omp master
					//{
					//std::cout << "erasing block" << std::endl;
					block = blocks.erase(block);
					//block->valid = false;
					//std::cout << "block erased" << std::endl;
					//}
					//#pragma omp barrier
						
					// The block iterator has now been advanced, so skip to the
					// beginning of the next iteration of the block loop
					continue;
				}
				else if (!block->has_smallest_extent[split_dimension]) // Otherwise, split the block in two along the largest dimension
				{
					//#pragma omp barrier
					//#pragma omp master
					//{
					/*std::cout << "splitting block" << std::endl;

					n = (block->start[split_dimension] + block->end[split_dimension])/2;
						
					if (n - block->start[split_dimension] >= smallest_block_extent &&
						block->end[split_dimension] - n >= smallest_block_extent)
					{
						// Split the block in two at this position
						ImageRectangle new_block(*block, split_dimension, n);
						new_blocks.push_back(new_block);
						block->setEnd(split_dimension, n);
								
						std::cout << "block splitted in " << ((split_dimension)? "y" : "x") << " at (" << block->start[split_dimension] << ", " << n << ", " << new_blocks.back().end[split_dimension] << ")" << std::endl;
					}*/

					const imp_float& half_of_total_pixel_error = total_pixel_error*0.5f;

					//splitted = false;

					accumulated_total_pixel_error = 0.0f;

					// Add up accumulated errors along the split dimension of the block
					for (n = block->start[split_dimension]; n < block->end[split_dimension]; n++)
					{
						accumulated_total_pixel_error += accumulated_pixel_errors[n];

						// Stop when we reach the point where the total error is equal on both sides
						if (accumulated_total_pixel_error > half_of_total_pixel_error)
						{
							if (n - block->start[split_dimension] >= smallest_block_extent &&
								block->end[split_dimension] - n >= smallest_block_extent)
							{
								// Split the block in two at this position
								new_blocks.emplace_back(*block, split_dimension, n);
								block->setEnd(split_dimension, n);
								
								std::cout << "block splitted in " << ((split_dimension)? "y" : "x") << " at (" << block->start[split_dimension] << ", " << n << ", " << new_blocks.back().end[split_dimension] << ")" << std::endl;
							}
							else
							{
								block->has_smallest_extent[split_dimension] = true;
							}
							//splitted = true;
							break;
						}
					}

					/*if (!splitted)
					{
						std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << "not splitted" << std::endl;
						throw;
					}*/

					//}
					//#pragma omp barrier
				}
			}
			//}

			//#pragma omp barrier 
			//#pragma omp master
			block++;
			//#pragma omp barrier
		}

		//#pragma omp barrier 
		//#pragma omp master
		//{
		//std::cout << "appending new blocks" << std::endl;
		if (!new_blocks.empty())
		{
			//for (std::vector<ImageRectangle>::const_iterator iter = new_blocks.begin(); iter != new_blocks.end(); iter++)
			//	blocks.push_back(*iter);

			//new_blocks.clear();
			blocks.splice(blocks.end(), new_blocks);
		}
		std::cout << "n_blocks = " << blocks.size() << std::endl;
		//}
		//#pragma omp barrier
	}
	//}
	
	delete accumulated_pixel_errors;

	secondary_image->stretch();
	image_util::writePPM("data/saved_snapshots/sample_counts.ppm", secondary_image->getRawPixelArray(), secondary_image->getWidth(), secondary_image->getHeight(), 3, Texture::ValueRange::COLOR_TEXTURE);

	delete secondary_image;
	
	if (gamma_encode)
		_image->gammaEncodeApprox(1.0f);

	_mesh_copies.clear();
}

Radiance Renderer::pathTrace(const Ray& ray, Medium& ray_medium, imp_uint scattering_count) const
{
	Radiance radiance = Radiance::black();
	Color attenuation = Color::white();
	SurfaceElement surface_element;

	imp_float distance = IMP_FLOAT_INF;

	if (scattering_count <= _max_scattering_count && findIntersectedSurface(ray, distance, surface_element))
	{
		// Include radiance emitted directly from the surface if this is the eye ray
		if (scattering_count == 0)
		{
			ray_medium.material = nullptr;

			if (surface_element.model->getMaterial()->isEmitter())
				radiance += surface_element.model->getMaterial()->getEmittedRadiance();
		}
		
		// The light travels in the opposite direction of the ray
		const Vector& scatter_direction = -ray.direction;
		
		if (ray_medium.material) // Are we inside an object?
		{
			// Attenuate radiance due to absorption in the medium
			attenuation = ray_medium.material->getAttenuationFactor(distance);

			// Flip the geometric normal so that it points out of the model like the shading normal
			surface_element.geometric.normal = -surface_element.geometric.normal;
		}
		else
		{
			// Compute directly scattered radiance on the surface
			radiance += getDirectlyScatteredRadianceFromPointLights(surface_element, scatter_direction);
			radiance += getDirectlyScatteredRadianceFromAreaLights(surface_element, scatter_direction);
		}
		
		// Include radiance scattered from other surfaces
		radiance += getScatteredRadianceFromSurface(surface_element, ray_medium, scatter_direction, scattering_count);
	}

	return attenuation*radiance;
}

bool Renderer::findIntersectedSurface(const Ray& ray, imp_float& distance, SurfaceElement& surface_element) const
{
	Geometry3D::MeshIntersectionData intersection_data;

	imp_float intersection_distance = _mesh_BVH.evaluateRayIntersection(_mesh_copies, ray, intersection_data);
	
	if (intersection_distance < distance)
	{
		distance = intersection_distance;
		
		const Model* model = _models->operator[](intersection_data.mesh_id);
        const TriangleMesh& mesh = _mesh_copies[intersection_data.mesh_id];

		surface_element.model = model;

		surface_element.geometric.position = ray(distance);
		surface_element.shading.position = surface_element.geometric.position;
		
		// Note: The geometric normal will point into or out of the model depending on the face orientation
		surface_element.geometric.normal = mesh.getFaceNormal(intersection_data.face_id);
		surface_element.shading.normal = mesh.getInterpolatedVertexNormal(intersection_data);

		surface_element.shading.color = Color::white();

		if (model->hasTexture())
		{
			surface_element.shading.texture_coordinate = mesh.getInterpolatedTextureCoordinates(intersection_data);

			if (model->hasColorTexture())
			{
				surface_element.computeTextureColor();
			}

			/*if (model->hasDisplacementMap())
			{
				surface_element.computeDisplacementMappedPosition();
			}*/

			if (surface_element.evaluateNormalMapping())
			{
				mesh.getInterpolatedVertexTangents(intersection_data, surface_element.shading.tangent, surface_element.shading.bitangent);

				surface_element.computeNormalMappedNormal();
			}
		}
		
		return true;
	}

	return false;
}

Geometry3D::Ray Renderer::getEyeRay(const Point2& pixel_center) const
{
    Vector unit_vector_from_camera_origin_to_pixel((pixel_center.x*_inverse_image_width - 0.5f)*_image_width_at_unit_distance_from_camera,
                                                   (pixel_center.y*_inverse_image_height - 0.5f)*_image_height_at_unit_distance_from_camera,
                                                   -1);

    unit_vector_from_camera_origin_to_pixel.normalize();

    return Ray((unit_vector_from_camera_origin_to_pixel*_near_plane_distance).toPoint(), unit_vector_from_camera_origin_to_pixel, _far_plane_distance);
}

Radiance Renderer::getDirectlyScatteredRadianceFromLights(const SurfaceElement& surface_element,
														  const Vector& scatter_direction) const
{
    return getDirectlyScatteredRadianceFromPointLights(surface_element, scatter_direction) +
		   getDirectlyScatteredRadianceFromDirectionalLights(surface_element, scatter_direction);
}

Radiance Renderer::getDirectlyScatteredRadianceFromPointLights(const SurfaceElement& surface_element,
															   const Vector& scatter_direction) const
{
    Radiance radiance = Color::black();

    Vector direction_to_source;
    imp_float squared_distance_to_source;
	imp_float cos_incoming_angle;

	Color attenuation_factor;

    for (std::vector<OmnidirectionalLight*>::const_iterator iter = _point_lights->begin(); iter != _point_lights->end(); iter++)
    {
        const OmnidirectionalLight* light = *iter;
		
		const Point& LOS_start_point = surface_element.geometric.position + _ray_origin_offset*surface_element.geometric.normal;

		attenuation_factor = Color::white();

        if (!(light->creates_shadows) || evaluateAttenuationAlongLineOfSight(LOS_start_point, light->getPosition(), attenuation_factor))
        {
			direction_to_source = light->getPosition() - surface_element.shading.position;
			squared_distance_to_source = direction_to_source.getSquaredLength();
			direction_to_source /= sqrt(squared_distance_to_source);

			cos_incoming_angle = direction_to_source.dot(surface_element.shading.normal);

			if (cos_incoming_angle > 0)
				radiance += surface_element.model->getMaterial()->evaluateFiniteBSDF(surface_element,
																					 direction_to_source,
																					 scatter_direction,
																					 cos_incoming_angle)
							*light->getTotalPower()
							*(cos_incoming_angle/(IMP_FOUR_PI*squared_distance_to_source))
							*attenuation_factor;
        }  
    }

    return radiance;
}

Radiance Renderer::getDirectlyScatteredRadianceFromDirectionalLights(const SurfaceElement& surface_element,
																     const Vector& scatter_direction) const
{
    Radiance radiance = Color::black();

	imp_float cos_incoming_angle;

    for (std::vector<DirectionalLight*>::const_iterator iter = _directional_lights->begin(); iter != _directional_lights->end(); iter++)
    {
        const DirectionalLight* light = *iter;
		const Vector& direction_to_source = -light->getDirection();

        if (!(light->creates_shadows) || sourceIsVisible(surface_element.geometric.position, direction_to_source, IMP_FLOAT_INF))
        {
			cos_incoming_angle = direction_to_source.dot(surface_element.shading.normal);

			if (cos_incoming_angle > 0)
				radiance += surface_element.model->getMaterial()->evaluateFiniteBSDF(surface_element,
																					 direction_to_source,
																					 scatter_direction,
																					 cos_incoming_angle)*light->getBiradiance()*cos_incoming_angle;
        }  
    }

    return radiance;
}

Radiance Renderer::getDirectlyScatteredRadianceFromAreaLights(const SurfaceElement& surface_element,
															  const Vector& scatter_direction) const
{
    Radiance radiance = Color::black();

    Vector direction_to_source;
    imp_float squared_distance_to_source;
	imp_float cos_incoming_angle, cos_emission_angle;
	Color attenuation_factor;

    for (std::vector<AreaLight*>::const_iterator iter = _area_lights->begin(); iter != _area_lights->end(); iter++)
    {
        const AreaLight* light = *iter;
        const SurfaceElement& source_surface_element = light->getRandomSurfaceElement();
		
		const Point& LOS_start_point = surface_element.geometric.position + surface_element.geometric.normal*_ray_origin_offset;
		const Point& LOS_end_point = source_surface_element.geometric.position + source_surface_element.geometric.normal*_ray_origin_offset;

		attenuation_factor = Color::white();

        if (!(light->creates_shadows) || evaluateAttenuationAlongLineOfSight(LOS_start_point, LOS_end_point, attenuation_factor))
        {   
			direction_to_source = source_surface_element.geometric.position - surface_element.shading.position;
            squared_distance_to_source = direction_to_source.getSquaredLength();
            direction_to_source /= sqrt(squared_distance_to_source);

			cos_incoming_angle = direction_to_source.dot(surface_element.shading.normal);

			if (cos_incoming_angle > 0)
			{
				cos_emission_angle = -(direction_to_source.dot(source_surface_element.geometric.normal));

				if (cos_emission_angle > 0)
					radiance += surface_element.model->getMaterial()->evaluateFiniteBSDF(surface_element,
																						 direction_to_source,
																						 scatter_direction,
																						 cos_incoming_angle)
								*light->getTotalPower()
								*(cos_incoming_angle*cos_emission_angle/(IMP_PI*squared_distance_to_source))
								*attenuation_factor;
			}
        }
    }
	
	Material::Impulse impulse;
	imp_float intersection_distance;
	SurfaceElement intersected_surface_element;

	if (surface_element.model->getMaterial()->getReflectiveBSDFImpulse(surface_element, scatter_direction, impulse))
	{
		const Ray& impulse_ray = Ray(surface_element.geometric.position, impulse.direction).nudge(_ray_origin_offset, surface_element.geometric.normal);

		intersection_distance = IMP_FLOAT_INF;

		if (findIntersectedSurface(impulse_ray, intersection_distance, intersected_surface_element))
		{
			radiance += (impulse.magnitude)*intersected_surface_element.model->getMaterial()->getEmittedRadiance();
		}
	}
	
	Medium ray_medium;
	Radiance transmitted_radiance = Radiance::black();
	Color attenuation = Color::white();

	intersected_surface_element = surface_element;

	while (intersected_surface_element.model->getMaterial()->getRefractiveBSDFImpulse(intersected_surface_element, ray_medium, scatter_direction, impulse))
	{
		const Ray& impulse_ray = Ray(intersected_surface_element.geometric.position, impulse.direction).nudge(-_ray_origin_offset, intersected_surface_element.geometric.normal);

		intersection_distance = IMP_FLOAT_INF;

		if (findIntersectedSurface(impulse_ray, intersection_distance, intersected_surface_element))
		{
			if (ray_medium.material)
			 	 attenuation *= ray_medium.material->getAttenuationFactor(intersection_distance);

			if (intersected_surface_element.model->getMaterial()->isEmitter())
				 transmitted_radiance += (impulse.magnitude)*intersected_surface_element.model->getMaterial()->getEmittedRadiance();
		}
		else
		{
			break;
		}
	}

	radiance += transmitted_radiance*attenuation;

    return radiance;
}

Radiance Renderer::getScatteredRadianceFromSurface(const SurfaceElement& surface_element,
												   Medium& ray_medium,
												   const Vector& scatter_direction,
												   imp_uint scattering_count) const
{
	Vector incoming_direction;
	Color weight;

	// Obtain a direction that light can be coming from and a weight for how its radiance 
	// should be modified (also keep track of whether we are inside a medium)
	if (surface_element.model->getMaterial()->scatter_back(surface_element, ray_medium, scatter_direction, incoming_direction, weight))
	{
		imp_float nudge_amount = _ray_origin_offset*math_util::sign(incoming_direction.dot(surface_element.geometric.normal));

		// Get the estimated radiance coming from that direction
		return weight*pathTrace(Ray(surface_element.geometric.position, incoming_direction).nudge(nudge_amount, surface_element.geometric.normal),
								ray_medium,
								++scattering_count);
	}

	return Radiance::black();
}

bool Renderer::sourceIsVisible(const Point& surface_point,
							   const Vector& direction_to_source,
						 	   imp_float distance_to_source) const
{   
	Geometry3D::MeshIntersectionData intersection_data;

    Ray shadow_ray(surface_point, direction_to_source, distance_to_source);

	return _mesh_BVH.evaluateRayIntersection(_mesh_copies, shadow_ray.nudge(_ray_origin_offset), intersection_data) >= distance_to_source || !(_models->operator[](intersection_data.mesh_id)->casts_shadows);
}

bool Renderer::clearLineOfSightTo(const Point& point) const
{   
	Geometry3D::MeshIntersectionData intersection_data;

	const Vector& direction = point - Point::origin();
	const imp_float max_distance = direction.getLength();

	return _mesh_BVH.evaluateRayIntersection(_mesh_copies, Ray(Point::origin(), direction/max_distance, max_distance), intersection_data) >= max_distance;
}

/*
bool Renderer::clearLineOfSightBetween(const SurfaceElement& surface_element,
									   const Point& end_point) const
{   
	imp_uint n_models = static_cast<imp_uint>(_models->size());
	Geometry3D::MeshIntersectionData intersection_data;

	const Point& start_point = surface_element.geometric.position + _ray_origin_offset*surface_element.geometric.normal;
	const Vector& direction = end_point - start_point;
	imp_float distance = direction.getLength();

	return _mesh_BVH.evaluateRayIntersection(_mesh_copies, Ray(start_point, direction/distance, distance), intersection_data) >= distance || !(_models->operator[](intersection_data.mesh_id)->casts_shadows);
}

bool Renderer::clearLineOfSightBetween(const SurfaceElement& surface_element_1,
									   const SurfaceElement& surface_element_2) const
{   
	imp_uint n_models = static_cast<imp_uint>(_models->size());
	Geometry3D::MeshIntersectionData intersection_data;

	const Point& point_1 = surface_element_1.geometric.position + surface_element_1.geometric.normal*_ray_origin_offset;
	const Point& point_2 = surface_element_2.geometric.position + surface_element_2.geometric.normal*_ray_origin_offset;

	const Vector& direction = point_2 - point_1;
	imp_float distance = direction.getLength();

	return _mesh_BVH.evaluateRayIntersection(_mesh_copies, Ray(point_1, direction/distance, distance), intersection_data) >= distance || !(_models->operator[](intersection_data.mesh_id)->casts_shadows);
}
*/

bool Renderer::evaluateAttenuationAlongLineOfSight(const Point& surface_point,
												   const Point& source_point,
												   Color& attenuation_factor) const
{   
	Geometry3D::MeshIntersectionData intersection_data;
	imp_float intersection_distance;
	const Material* current_material = nullptr;

	imp_float total_intersection_distance = 0;

	const Vector& direction = source_point - surface_point;
	const imp_float max_distance = direction.getLength();

	Ray LOS(surface_point, direction/max_distance, max_distance);

	while (true)
	{
		intersection_distance = _mesh_BVH.evaluateRayIntersection(_mesh_copies, LOS, intersection_data);

		if (intersection_distance == IMP_FLOAT_INF)
			break; // Ray did not hit any obstructions

		intersection_distance += 10*_ray_origin_offset;

		// Update total distance travelled along ray
		total_intersection_distance += intersection_distance;

		LOS.origin = LOS(intersection_distance);
		
		if (total_intersection_distance >= max_distance)
			break; // We have passed the light source

		if (!(_models->operator[](intersection_data.mesh_id)->casts_shadows))
			continue; // The obstruction does not cast shadows and should be ignored

		const Material* material = _models->operator[](intersection_data.mesh_id)->getMaterial();

		if (material->isTransparent())
		{
			if (material == current_material)
			{
				// We have reached the other side of the transparent model
				current_material = nullptr;

				// Update attenuation
				attenuation_factor *= material->getAttenuationFactor(intersection_distance)*material->getTransmittance();
			}
			else
			{
				// We have entered a transparent model
				current_material = material;
				
				// Update attenuation
				attenuation_factor *= material->getTransmittance();
			}
		}
		else
		{
			// We have reached an opaque model
			return false;
		}
	}

	return true;
}

void Renderer::computeRadianceAtAllVertices(TriangleMesh& mesh, const SurfaceElement& surface_element)
{
    assert(mesh.isHomogenized());
    assert(mesh.hasVertexNormals());

    int n_vertices = static_cast<int>(mesh.getNumberOfVertices());
    int idx;
	imp_float vertex_camera_distance;
	SurfaceElement surface_element_copy = surface_element;
	
	mesh.initializeVertexData3();

    #pragma omp parallel for default(shared) \
							 firstprivate(surface_element_copy) \
                             private(idx, vertex_camera_distance) \
                             shared(mesh, n_vertices) \
							 schedule(static) \
                             if (use_omp)
    for (idx = 0; idx < n_vertices; idx++)
    {
		surface_element_copy.geometric.position = mesh.getVertex(idx);
		surface_element_copy.shading.position = surface_element_copy.geometric.position;
		
		surface_element_copy.shading.normal = mesh.getVertexNormal(idx);
		surface_element_copy.geometric.normal = surface_element_copy.shading.normal;

		surface_element_copy.shading.color = Color::white();

        Vector& scatter_direction = _camera_cframe.origin - surface_element_copy.shading.position;
		vertex_camera_distance = scatter_direction.getLength();

		if (vertex_camera_distance > 0)
		{
			scatter_direction /= vertex_camera_distance;

			const Radiance& radiance = getDirectlyScatteredRadianceFromLights(surface_element_copy,
																			  scatter_direction).clamp();

			mesh.setVertexData3(idx, radiance.r, radiance.g, radiance.b);
		}
    }
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
		mesh.getVertexData3ForFace(face_idx, vertex_color_A, vertex_color_B, vertex_color_C);

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

void Renderer::drawEdgesInImageSpace(const TriangleMesh& mesh, imp_float luminance) const
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

void Renderer::drawEdgesInCameraSpace(const TriangleMesh& mesh, imp_float luminance) const
{
    imp_uint n_faces = mesh.getNumberOfFaces();
	Point face_vertices[3];
	bool vertex_1_visible, vertex_2_visible, vertex_3_visible;

    for (imp_uint face_idx = 0; face_idx < n_faces; face_idx++)
    {
		mesh.getFaceVertices(face_idx, face_vertices);

		const Vector& bump = mesh.getFaceNormal(face_idx)*2*_ray_origin_offset;

		vertex_1_visible = clearLineOfSightTo(face_vertices[0] + bump);
		vertex_2_visible = clearLineOfSightTo(face_vertices[1] + bump);
		vertex_3_visible = clearLineOfSightTo(face_vertices[2] + bump);

		if (vertex_1_visible + vertex_2_visible + vertex_3_visible <= 1)
			continue;

		const Triangle2& face = mesh.getProjectedFace(face_idx,
													  _image_width,
													  _image_height,
													  _inverse_image_width_at_unit_distance_from_camera,
													  _inverse_image_height_at_unit_distance_from_camera);

		const Point2 vertex_1 = face.getPointA();
		const Point2 vertex_2 = face.getPointB();
		const Point2 vertex_3 = face.getPointC();

		vertex_1_visible = vertex_1_visible &&
						   vertex_1.x >= 0 && vertex_1.x < _image_width &&
						   vertex_1.y >= 0 && vertex_1.y < _image_height;

		vertex_2_visible = vertex_2_visible &&
						   vertex_2.x >= 0 && vertex_2.x < _image_width &&
						   vertex_2.y >= 0 && vertex_2.y < _image_height;

		vertex_3_visible = vertex_3_visible &&
						   vertex_3.x >= 0 && vertex_3.x < _image_width &&
						   vertex_3.y >= 0 && vertex_3.y < _image_height;

		if (vertex_1_visible && vertex_2_visible)
		{
			_image->drawLine(vertex_1.x, vertex_1.y,
							 vertex_2.x, vertex_2.y, luminance);
		}
		
		if (vertex_2_visible && vertex_3_visible)
		{
			_image->drawLine(vertex_2.x, vertex_2.y,
							 vertex_3.x, vertex_3.y, luminance);
		}
        
		if (vertex_3_visible && vertex_1_visible)
		{
			_image->drawLine(vertex_3.x, vertex_3.y,
							 vertex_1.x, vertex_1.y, luminance);
		}
    }
}

void Renderer::printPickInfo()
{
	if (!pixel_was_picked)
		return;

	assert(picked_x < _image->getWidth() && picked_y < _image->getHeight());

	const Ray& eye_ray = getEyeRay(Point2(static_cast<imp_float>(picked_x) + 0.5f, static_cast<imp_float>(_image->getHeight() - picked_y - 1) + 0.5f));

	/*Geometry3D::MeshIntersectionData intersection_data;
	imp_float intersection_distance = _mesh_BVH.evaluateRayIntersection(_mesh_copies, eye_ray, intersection_data);

	if (intersection_distance < IMP_FLOAT_INF)
	{
		std::cout << intersection_distance << " " << intersection_data.mesh_id << " " << intersection_data.face_id << std::endl;
	}
	else
	{
		std::cout << intersection_distance << std::endl;
	}*/

	Medium ray_medium;
	
	pathTrace(eye_ray, ray_medium, 0);

	pixel_was_picked = false;
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
