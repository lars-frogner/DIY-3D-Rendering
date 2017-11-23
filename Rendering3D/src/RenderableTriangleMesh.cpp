#include "RenderableTriangleMesh.hpp"
#include "string_util.hpp"
#include "Vertex3.hpp"
#include "Scene.hpp"
#include "BAHNode.hpp"
#include <omp.h>
#include <cstdlib>
#include <cmath>
#include <map>
#include <utility>
#include <fstream>
#include <iostream>
#include <sstream>

namespace Impact {
namespace Rendering3D {

RenderableTriangleMesh::RenderableTriangleMesh()
    : Geometry3D::TriangleMesh(),
      _has_material(false),
	  is_dynamic(false),
	  uses_direct_lighting(true),
	  casts_shadows(false),
	  render_faces(true),
	  render_edges(false),
	  remove_hidden_faces(true),
	  perform_clipping(true),
	  is_visible(true) {}

RenderableTriangleMesh::RenderableTriangleMesh(imp_uint n_vertices, imp_float vertex_array[],
											   imp_uint n_faces, imp_uint face_array[])
    : Geometry3D::TriangleMesh(n_vertices, vertex_array, n_faces, face_array),
      _has_material(false),
	  is_dynamic(false),
	  uses_direct_lighting(true),
	  casts_shadows(false),
	  render_faces(true),
	  render_edges(false),
	  remove_hidden_faces(true),
	  perform_clipping(true),
	  is_visible(true) {}

RenderableTriangleMesh::RenderableTriangleMesh(const RenderableTriangleMesh& mesh_1,
											   const RenderableTriangleMesh& mesh_2)
    : Geometry3D::TriangleMesh(mesh_1, mesh_2),
      _has_material(false),
	  is_dynamic(false),
	  uses_direct_lighting(true),
	  casts_shadows(false),
	  render_faces(true),
	  render_edges(false),
	  remove_hidden_faces(true),
	  perform_clipping(true),
	  is_visible(true) {}

RenderableTriangleMesh::RenderableTriangleMesh(const TriangleMesh& other)
    : Geometry3D::TriangleMesh(other),
      _has_material(false),
	  is_dynamic(false),
	  uses_direct_lighting(true),
	  casts_shadows(false),
	  render_faces(true),
	  render_edges(false),
	  remove_hidden_faces(true),
	  perform_clipping(true),
	  is_visible(true) {}

RenderableTriangleMesh::RenderableTriangleMesh(const RenderableTriangleMesh& other)
    : Geometry3D::TriangleMesh(other),
      _colors(other._colors),
      _has_material(other._has_material),
	  is_dynamic(other.is_dynamic),
	  uses_direct_lighting(other.uses_direct_lighting),
	  casts_shadows(other.casts_shadows),
	  render_faces(other.render_faces),
	  render_edges(other.render_edges),
	  remove_hidden_faces(other.remove_hidden_faces),
	  perform_clipping(other.perform_clipping),
	  is_visible(other.is_visible)
{
    if (other._material)
    {
        if (BlinnPhongMaterial* material_ptr = dynamic_cast<BlinnPhongMaterial*>(other._material))
            _material = new BlinnPhongMaterial(*material_ptr);
    }
}

RenderableTriangleMesh::~RenderableTriangleMesh()
{
    if (_material) delete _material;
}

RenderableTriangleMesh RenderableTriangleMesh::operator=(const RenderableTriangleMesh& other)
{
	return RenderableTriangleMesh(other);
}

RenderableTriangleMesh RenderableTriangleMesh::file(const std::string& filename)
{
	string_vec material_names;
    string_vec material_files;
 
	RenderableTriangleMesh mesh(TriangleMesh::file(filename, material_files, material_names));

	imp_uint n_material_files = static_cast<imp_uint>(material_files.size());
	imp_uint n_material_names = static_cast<imp_uint>(material_names.size());

    // Materials

    assert(n_material_names <= 1);

    imp_uint n_materials;
    imp_uint n;
	imp_uint idx;

    for (n = 0; n < n_material_files; n++)
    {
        const std::vector<BlinnPhongMaterial>& materials = _getMtlFileData(material_files[n]);
        n_materials = static_cast<imp_uint>(materials.size());
    
        for (idx = 0; idx < n_materials; idx++)
        {
            if (materials[idx].getName() == material_names[0])
            {
                mesh.setMaterial(materials[idx]);
                break;
            }
        }

        if (mesh._has_material) break;
    }

    return mesh;
}

std::vector<BlinnPhongMaterial> RenderableTriangleMesh::_getMtlFileData(const std::string& filename)
{
    imp_uint length = static_cast<imp_uint>(filename.length());
    assert(length > 4 &&
           filename.substr(length-4, 4) == ".mtl");

    std::ifstream infile(filename);
    assert(infile);

    std::string line, word, current_material = "";
    imp_uint split;
    bool has_material = false;

    std::map<std::string, std::map<std::string, std::string> > material_properties;

    while (std::getline(infile, line))
    {
        string_util::trim(line);

        split = static_cast<imp_uint>(line.find(" "));

        if (split == static_cast<imp_uint>(std::string::npos))
            continue;

        word = line.substr(0, split);

        if (word == "#")
        {
            continue;
        }
        else if (word == "newmtl")
        {
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            assert(line.length() > 0);
            
            if (material_properties.find(line) == material_properties.end())
                material_properties.insert(std::make_pair(line, std::map<std::string, std::string>()));

            current_material = line;
            has_material = true;
        }
        else
        {
            assert(has_material);

            std::map<std::string, std::string>& properties = material_properties[current_material];
            
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            assert(line.length() > 0);

            if (properties.find(line) == properties.end())
                properties.insert(std::make_pair(word, line));
            else
                properties[word] = line;
        }
    }

    std::vector<BlinnPhongMaterial> materials;

    string_vec splitted;
    imp_uint n_values;

    Reflectance diffuse_reflectance;
    Reflectance glossy_reflectance;
    float smoothness;

    std::map<std::string, std::map<std::string, std::string> >::const_iterator iter;
    std::map<std::string, std::string>::const_iterator property_iter;

    for (iter = material_properties.begin(); iter != material_properties.end(); iter++)
    {
        const std::map<std::string, std::string>& properties = iter->second;

        
        diffuse_reflectance = Reflectance::black();
        glossy_reflectance = Reflectance::black();
        smoothness = 0.0f;

        for (property_iter = properties.begin(); property_iter != properties.end(); property_iter++)
        {
            if (property_iter->first == "Kd")
            {
                splitted = string_util::split(property_iter->second);
                n_values = static_cast<imp_uint>(splitted.size());
                assert(n_values == 3);

                diffuse_reflectance.r = static_cast<imp_float>(atof(splitted[0].c_str()));
                diffuse_reflectance.g = static_cast<imp_float>(atof(splitted[1].c_str()));
                diffuse_reflectance.b = static_cast<imp_float>(atof(splitted[2].c_str()));
            }
            else if (property_iter->first == "Ks")
            {
                splitted = string_util::split(property_iter->second);
                n_values = static_cast<imp_uint>(splitted.size());
                assert(n_values == 3);

                glossy_reflectance.r = static_cast<imp_float>(atof(splitted[0].c_str()));
                glossy_reflectance.g = static_cast<imp_float>(atof(splitted[1].c_str()));
                glossy_reflectance.b = static_cast<imp_float>(atof(splitted[2].c_str()));
            }
            else if (property_iter->first == "Ns")
            {
                splitted = string_util::split(property_iter->second);
                n_values = static_cast<imp_uint>(splitted.size());
                assert(n_values == 1);

                smoothness = static_cast<imp_float>(atof(splitted[0].c_str()));
            }
        }

        materials.push_back(BlinnPhongMaterial(diffuse_reflectance,
                                               glossy_reflectance,
                                               smoothness,
                                               iter->first));
    }

    return materials;
}

RenderableTriangleMesh RenderableTriangleMesh::triangle(const Triangle& triangle_obj)
{
    return RenderableTriangleMesh(TriangleMesh::triangle(triangle_obj));
}

RenderableTriangleMesh RenderableTriangleMesh::box(const Box& box_obj)
{
    return RenderableTriangleMesh(TriangleMesh::box(box_obj));
}

RenderableTriangleMesh RenderableTriangleMesh::room(const Box& box_obj)
{
    return RenderableTriangleMesh(TriangleMesh::room(box_obj));
}

RenderableTriangleMesh RenderableTriangleMesh::sheet(const Point& origin, const Vector& width_vector, const Vector& height_vector)
{
    return RenderableTriangleMesh(TriangleMesh::sheet(origin, width_vector, height_vector));
}

RenderableTriangleMesh RenderableTriangleMesh::sphere(const Sphere& sphere_obj, imp_uint resolution)
{
	return RenderableTriangleMesh(TriangleMesh::sphere(sphere_obj, resolution));
}

void RenderableTriangleMesh::setMaterial(const BlinnPhongMaterial& material)
{
    if (!_has_material)
    {
        _material = new BlinnPhongMaterial(material);
        _has_material = true;
    }
}

void RenderableTriangleMesh::setMaterialReplaceOld(const BlinnPhongMaterial& material)
{
    if (_material) delete _material;
    _material = new BlinnPhongMaterial(material);
    _has_material = true;
}

RenderableTriangleMesh RenderableTriangleMesh::withMaterial(const BlinnPhongMaterial& material)
{
    RenderableTriangleMesh mesh(*this);
	mesh.setMaterial(material);
	return mesh;
}

RenderableTriangleMesh RenderableTriangleMesh::withMaterialReplaceOld(const BlinnPhongMaterial& material)
{
    RenderableTriangleMesh mesh(*this);
	mesh.setMaterialReplaceOld(material);
	return mesh;
}

void RenderableTriangleMesh::shadeVerticesDirect(const Scene& scene)
{
    assert(_is_homogenized);
    assert(_has_normals);
    assert(_has_material);

    Point vertex_point;
    Vector normal_vector;
    int n_vertices = static_cast<int>(getNumberOfVertices());
    int idx;

    _colors.resize(n_vertices);

    #pragma omp parallel for default(shared) \
                             private(idx, vertex_point, normal_vector) \
                             shared(scene, n_vertices) \
							 schedule(static) \
                             if (uses_omp)
    for (idx = 0; idx < n_vertices; idx++)
    {
        vertex_point.x = _vertices(0, idx);
        vertex_point.y = _vertices(1, idx);
        vertex_point.z = _vertices(2, idx);

        normal_vector.x = _normals(0, idx);
        normal_vector.y = _normals(1, idx);
        normal_vector.z = _normals(2, idx);

        const Vector& scatter_direction = scene._camera_position - vertex_point;

        const Radiance& radiance = scene._getRadiance(vertex_point,
                                                      normal_vector,
                                                      scatter_direction,
                                                      _material).clamp();

        _colors[idx] = radiance;
    }
}

Geometry2D::Triangle RenderableTriangleMesh::getProjectedFace(const Scene& scene, imp_uint face_idx) const
{
    return Triangle2(scene._getPerspectiveProjected(_vertices.col(_faces(0, face_idx)).head(3)),
                     scene._getPerspectiveProjected(_vertices.col(_faces(1, face_idx)).head(3)),
                     scene._getPerspectiveProjected(_vertices.col(_faces(2, face_idx)).head(3)));
}

void RenderableTriangleMesh::getVertexAttributes(imp_uint face_idx, Point vertices[3], Vector normals[3], Material*& material) const
{
    assert(_has_material);

    TriangleMesh::getVertexAttributes(face_idx, vertices, normals);
    
    material = _material;
}

void RenderableTriangleMesh::computeBoundingAreaHierarchy(const Scene& scene)
{
    imp_uint n_triangles = getNumberOfFaces();
    std::vector< AABRContainer > objects(n_triangles);

    AxisAlignedRectangle aabr(Point2::max(), Point2::min());

    for (imp_uint face_idx = 0; face_idx < n_triangles; face_idx++)
    {
        const Triangle2& projected_face = getProjectedFace(scene, face_idx);
        objects[face_idx].aabr = projected_face.getAABR();
        aabr.merge(objects[face_idx].aabr);
        objects[face_idx].centroid = projected_face.getCentroid();
        objects[face_idx].id = face_idx;
    }

    _bounding_area_hierarchy = BoundingAreaHierarchy(aabr, objects);

    objects.clear();
}

std::vector<imp_uint> RenderableTriangleMesh::getIntersectedFaceIndices(const Point2& pixel_center) const
{
    return _bounding_area_hierarchy.getIntersectedObjectIDs(pixel_center);
}

bool RenderableTriangleMesh::sampleRadianceFromFace(const Scene& scene, const Ray& ray, imp_uint face_idx, Radiance& pixel_radiance, imp_float& closest_distance) const
{
    assert(_has_normals);
    assert(_has_material);

    imp_float alpha, beta, gamma;
    imp_float distance = evaluateRayFaceIntersection(ray, face_idx, alpha, beta, gamma);

    if (distance >= closest_distance)
        return false;

    closest_distance = distance;

    const Point& intersection_point = ray(distance);

    imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);
    Vector interpolated_normal(_normals(0, i)*alpha + _normals(0, j)*beta + _normals(0, k)*gamma,
                               _normals(1, i)*alpha + _normals(1, j)*beta + _normals(1, k)*gamma,
                               _normals(2, i)*alpha + _normals(2, j)*beta + _normals(2, k)*gamma);
    interpolated_normal.normalize();

    pixel_radiance = scene._getRadiance(intersection_point,
                                        interpolated_normal,
                                        -ray.direction,
                                        _material);

    return true;
}

bool RenderableTriangleMesh::allZAbove(imp_float z_low) const
{
	return arma::all(_vertices.row(2) > z_low);
}

bool RenderableTriangleMesh::isInsideParallelViewVolume() const
{
	AxisAlignedBox parallel_view_volume(Point(-1, -1, -1), Point(1, 1, 0));
	return (_aabb.intersects(parallel_view_volume)) && (!_aabb.encloses(parallel_view_volume));
}

void RenderableTriangleMesh::clipNearPlaneAt(imp_float z_near)
{
    assert(_is_homogenized);
    _clip(2, z_near, 1); // Clip away points with z > z_near
}

void RenderableTriangleMesh::clipLeftPlane()
{
    assert(_is_homogenized);
    _clip(0, -1, -1); // Clip away points with x < -1
}

void RenderableTriangleMesh::clipRightPlane()
{
    assert(_is_homogenized);
    _clip(0, 1, 1); // Clip away points with x > 1
}

void RenderableTriangleMesh::clipBottomPlane()
{
    assert(_is_homogenized);
    _clip(1, -1, -1); // Clip away points with y < -1
}

void RenderableTriangleMesh::clipTopPlane()
{
    assert(_is_homogenized);
    _clip(1, 1, 1); // Clip away points with y > 1
}

void RenderableTriangleMesh::clipNearPlane()
{
    _clip(2, 0, 1); // Clip away points with z > 0
}

void RenderableTriangleMesh::clipFarPlane()
{
    assert(_is_homogenized);
    _clip(2, -1, -1); // Clip away points with z < -1
}

void RenderableTriangleMesh::clipNonNearPlanes()
{
    const Point& lower_corner = _aabb.lower_corner;
    const Point& upper_corner = _aabb.upper_corner;

    if (lower_corner.x < -1) clipLeftPlane();
    if (upper_corner.x >  1) clipRightPlane();
    if (lower_corner.y < -1) clipBottomPlane();
    if (upper_corner.y >  1) clipTopPlane();
    if (lower_corner.z < -1) clipFarPlane();
}

void RenderableTriangleMesh::removeBackwardFacingFaces()
{
    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k;
    imp_float AB_x, AB_y, AC_x, AC_y;

    for (int idx = static_cast<int>(n_faces) - 1; idx >= 0; idx--)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        AB_x = _vertices(0, j) - _vertices(0, i);
        AB_y = _vertices(1, j) - _vertices(1, i);
        AC_x = _vertices(0, k) - _vertices(0, i);
        AC_y = _vertices(1, k) - _vertices(1, i);

        if (AB_x*AC_y - AB_y*AC_x < 0)
        {
            removeFace(idx);
        }
    }
}

void RenderableTriangleMesh::_clip(imp_uint component, imp_float limit, int sign)
{
    assert(component < 3);
    assert(sign == 1 || sign == -1);

    imp_uint n_vertices = getNumberOfVertices();
    assert(_colors.empty() || _colors.size() == n_vertices);

    imp_uint component_1 = (component + 1) % 3;
    imp_uint component_2 = (component + 2) % 3;

    imp_float signed_limit = sign*limit;

    imp_uint i, j, k;
    imp_float A[3], B[3], C[3];

    bool A_outside, B_outside, C_outside;

    imp_uint n_outside;
    imp_uint inside_idx_1, inside_idx_2;

    imp_uint n_faces = getNumberOfFaces();
    imp_uint last_vertex = n_vertices - 1;
    
    std::vector<imp_uint> deleted_faces;

    for (imp_uint idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);
        
        A[0] = _vertices(0, i); A[1] = _vertices(1, i); A[2] = _vertices(2, i);
        B[0] = _vertices(0, j); B[1] = _vertices(1, j); B[2] = _vertices(2, j);
        C[0] = _vertices(0, k); C[1] = _vertices(1, k); C[2] = _vertices(2, k);

        A_outside = sign*A[component] > signed_limit;
        B_outside = sign*B[component] > signed_limit;
        C_outside = sign*C[component] > signed_limit;

        n_outside = 0;
        if (A_outside) n_outside++;
        if (B_outside) n_outside++;
        if (C_outside) n_outside++;

		if (i == j || i == k || j == k)
			n_outside = 3;

        if (n_outside > 0)
        {
            if (n_outside == 1)
            {
                if (A_outside)
                {
                    inside_idx_1 = j;
                    inside_idx_2 = k;
                    _addIntersectionVertices(i, j, k,
                                             A, B, C,
                                             component, component_1, component_2,
                                             limit);
                }
                else if (B_outside)
                {
                    inside_idx_1 = k;
                    inside_idx_2 = i;
                    _addIntersectionVertices(i, j, k,
                                             B, C, A,
                                             component, component_1, component_2,
                                             limit);
                }
                else
                {
                    inside_idx_1 = i;
                    inside_idx_2 = j;
                    _addIntersectionVertices(i, j, k,
                                             C, A, B,
                                             component, component_1, component_2,
                                             limit);
                }
            
                _faces(0, idx) = inside_idx_1;
                _faces(1, idx) = inside_idx_2;
                _faces(2, idx) = last_vertex + 1;

                _faces.insert_cols(_faces.n_cols, arma::Col<imp_uint>({last_vertex + 1, inside_idx_2, last_vertex + 2}));
                
                last_vertex += 2;
            }
            else if (n_outside == 2)
            {
                if (!A_outside) 
                {
                    inside_idx_1 = i;
                    _addIntersectionVertices(i, j, k,
                                             A, B, C,
                                             component, component_1, component_2,
                                             limit);
                }
                else if (!B_outside)
                {
                    inside_idx_1 = j;
                    _addIntersectionVertices(i, j, k,
                                             B, C, A,
                                             component, component_1, component_2,
                                             limit);
                }
                else                
                {
                    inside_idx_1 = k;
                    _addIntersectionVertices(i, j, k,
                                             C, A, B,
                                             component, component_1, component_2,
                                             limit);
                }
            
                _faces(0, idx) = inside_idx_1;
                _faces(1, idx) = last_vertex + 1;
                _faces(2, idx) = last_vertex + 2;
                
                last_vertex += 2;
            }
            else
            {
                deleted_faces.push_back(idx);
            }
        }
    }

    while (!deleted_faces.empty())
    {
        _faces.shed_col(deleted_faces.back());
        deleted_faces.pop_back();
    }
}

void RenderableTriangleMesh::_addIntersectionVertices(imp_uint i, imp_uint j, imp_uint k,
													  imp_float origin[], imp_float other_1[], imp_float other_2[],
													  imp_uint component_0, imp_uint component_1, imp_uint component_2,
													  imp_float limit)
{
    // Find vertices

    arma::Col<imp_float> vertex_vector1(4);
    arma::Col<imp_float> vertex_vector2(4);
    vertex_vector1(3) = 1;
    vertex_vector2(3) = 1;

    imp_float orig_plane_dist = limit - origin[component_0];

    imp_float scaling = orig_plane_dist/(other_1[component_0] - origin[component_0]);
    vertex_vector1(component_0) = limit;
    vertex_vector1(component_1) = origin[component_1] + scaling*(other_1[component_1] - origin[component_1]);
    vertex_vector1(component_2) = origin[component_2] + scaling*(other_1[component_2] - origin[component_2]);

    scaling = orig_plane_dist/(other_2[component_0] - origin[component_0]);
    vertex_vector2(component_0) = limit;
    vertex_vector2(component_1) = origin[component_1] + scaling*(other_2[component_1] - origin[component_1]);
    vertex_vector2(component_2) = origin[component_2] + scaling*(other_2[component_2] - origin[component_2]);

    imp_float alpha1, beta1, gamma1;
    imp_float alpha2, beta2, gamma2;

    // Find barycentric coordinates of new vertices in old triangle
    if (_has_normals || !_colors.empty())
    {
        Triangle face(Point(_vertices(0, i), _vertices(1, i), _vertices(2, i)),
                      Point(_vertices(0, j), _vertices(1, j), _vertices(2, j)),
                      Point(_vertices(0, k), _vertices(1, k), _vertices(2, k)));

        face.computeNormals();
        
        Point vertex1(vertex_vector1(0), vertex_vector1(1), vertex_vector1(2));
        Point vertex2(vertex_vector2(0), vertex_vector2(1), vertex_vector2(2));

        face.getBarycentricCoordinatesInside(vertex1, alpha1, beta1, gamma1);
        face.getBarycentricCoordinatesInside(vertex2, alpha2, beta2, gamma2);
    }

    // Add normals
    if (_has_normals)
    {
        _normals.insert_cols(_normals.n_cols, arma::normalise(alpha1*_normals.col(i) + beta1*_normals.col(j) + gamma1*_normals.col(k)));
        _normals.insert_cols(_normals.n_cols, arma::normalise(alpha2*_normals.col(i) + beta2*_normals.col(j) + gamma2*_normals.col(k)));
    }
    
    // Add colors
    if (!_colors.empty())
    {
        const Color& color_A = _colors[i];
        const Color& color_B = _colors[j];
        const Color& color_C = _colors[k];

        const Color& color1 = alpha1*color_A + beta1*color_B + gamma1*color_C;
        const Color& color2 = alpha2*color_A + beta2*color_B + gamma2*color_C;

        _colors.push_back(color1);
        _colors.push_back(color2);
    }
    
    _vertices.insert_cols(_vertices.n_cols, vertex_vector1);
    _vertices.insert_cols(_vertices.n_cols, vertex_vector2);
}

void RenderableTriangleMesh::drawEdges(Image& image, float luminance) const
{
    assert(_is_homogenized);

    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k;

    for (imp_uint idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        image.drawLine(_vertices(0, i), _vertices(1, i),
                       _vertices(0, j), _vertices(1, j), luminance);

        image.drawLine(_vertices(0, j), _vertices(1, j),
                       _vertices(0, k), _vertices(1, k), luminance);
        
        image.drawLine(_vertices(0, k), _vertices(1, k),
                       _vertices(0, i), _vertices(1, i), luminance);
    }
}

void RenderableTriangleMesh::drawFaces(Image& image) const
{
    assert(_is_homogenized);
    assert(_colors.size() == getNumberOfVertices());

    int n_faces = static_cast<int>(getNumberOfFaces());
    int idx;
    imp_uint i, j, k;
    
    #pragma omp parallel for default(shared) \
                             private(idx, i, j, k) \
                             shared(image, n_faces) \
							 schedule(dynamic) \
                             if (uses_omp)
    for (idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        image.drawTriangle(Vertex3(_vertices(0, i), _vertices(1, i), _vertices(2, i), _colors[i]),
                           Vertex3(_vertices(0, j), _vertices(1, j), _vertices(2, j), _colors[j]),
                           Vertex3(_vertices(0, k), _vertices(1, k), _vertices(2, k), _colors[k]));
    }
}

void RenderableTriangleMesh::drawFaces(Image& image, Color color) const
{
    assert(_is_homogenized);

    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k;

    for (imp_uint idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        image.drawTriangle(Vertex3(_vertices(0, i), _vertices(1, i), _vertices(2, i), color),
                           Vertex3(_vertices(0, j), _vertices(1, j), _vertices(2, j), color),
                           Vertex3(_vertices(0, k), _vertices(1, k), _vertices(2, k), color));
    }
}

void RenderableTriangleMesh::saveAs(const std::string& filename, const std::string& material_filename /* = std::string() */) const
{
    imp_uint length = static_cast<imp_uint>(filename.length());
    assert(length > 4 &&
           filename.substr(length-4, 4) == ".obj");

    std::ofstream outfile(filename);
    assert(outfile);

    std::string material_name = "default";

    bool include_material = _has_material && !material_filename.empty();

    if (include_material)
    {
        length = static_cast<imp_uint>(material_filename.length());
        assert(length > 4 &&
               material_filename.substr(length-4, 4) == ".mtl");

        material_name = _material->getName();

        outfile << "mtllib " << material_filename << std::endl << std::endl;
    }

    imp_uint n_vertices = getNumberOfVertices();
    imp_uint n_faces = getNumberOfFaces();
    imp_uint idx;

    for (idx = 0; idx < n_vertices; idx++)
    {
        outfile << "v " << _vertices(0, idx) << " " << 
                           _vertices(1, idx) << " " <<
                           _vertices(2, idx) << " " <<
                           _vertices(3, idx) <<
                           std::endl;
    }

    outfile << std::endl;

    if (_has_normals)
    {
        for (idx = 0; idx < n_vertices; idx++)
        {
            outfile << "vn " << _normals(0, idx) << " " << 
                                _normals(1, idx) << " " <<
                                _normals(2, idx) <<
                                std::endl;
        }
        
        outfile << std::endl;

        if (include_material)
        {
            outfile << "g Object" << std::endl;
            outfile << "usemtl " << material_name << std::endl;
        }

        for (idx = 0; idx < n_faces; idx++)
        {
            outfile << "f " << _faces(0, idx)+1 << "//" << _faces(0, idx)+1 << " " <<
                               _faces(1, idx)+1 << "//" << _faces(1, idx)+1 << " " <<
                               _faces(2, idx)+1 << "//" << _faces(2, idx)+1 << std::endl;
        }
    }
    else
    {
        if (include_material)
        {
            outfile << "g Object" << std::endl;
            outfile << "usemtl " << material_name << std::endl;
        }

        for (idx = 0; idx < n_faces; idx++)
        {
            outfile << "f " << _faces(0, idx)+1 << " " << 
                               _faces(1, idx)+1 << " " <<
                               _faces(2, idx)+1 << std::endl;
        }
    }

    outfile.close();
}

bool RenderableTriangleMesh::hasMaterial() const
{
    return _has_material;
}

} // Rendering3D
} // Impact
