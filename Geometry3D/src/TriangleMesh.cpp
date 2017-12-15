#include "TriangleMesh.hpp"
#include "string_util.hpp"
#include "BAHNode.hpp"
#include <cstdlib>
#include <cmath>
#include <map>
#include <utility>
#include <fstream>
#include <iostream>
#include <sstream>

namespace Impact {
namespace Geometry3D {

TriangleMesh::TriangleMesh()
	: _is_homogenized(true),
      _has_vertex_normals(false),
      _has_face_normals(false),
	  _has_texture_coordinates(false),
	  _has_vertex_tangents(false),
	  _has_vertex_data_3(false),
      _has_aabb(false) {}

TriangleMesh TriangleMesh::file(const std::string& filename, string_vec& material_files, string_vec& material_names)
{
    string_vec vertices;
    string_vec texture_coords;
    string_vec vertex_normals;
    string_vec param_space_vertices;
    string_vec faces;
    int_vec face_material_indices;

    _getObjFileData(filename,
                    vertices,
                    texture_coords,
                    vertex_normals,
                    param_space_vertices,
                    faces,
                    material_files,
                    material_names,
                    face_material_indices);

    imp_uint n_vertices = static_cast<imp_uint>(vertices.size());
    imp_uint n_texture_coords = static_cast<imp_uint>(texture_coords.size());
    imp_uint n_vertex_normals = static_cast<imp_uint>(vertex_normals.size());
    imp_uint n_param_space_vertices = static_cast<imp_uint>(param_space_vertices.size());
    imp_uint n_faces = static_cast<imp_uint>(faces.size());
    imp_uint idx;
    imp_uint n_values;
    string_vec splitted;
    arma::Col<imp_uint> extra_face(6);

    TriangleMesh mesh;

    // Vertices

    mesh._vertices = arma::Mat<imp_float>(4, n_vertices);
    mesh._vertices.row(3).ones();
    mesh._is_homogenized = true;

    for (idx = 0; idx < n_vertices; idx++)
    {
        splitted = string_util::split(vertices[idx]);
        n_values = static_cast<imp_uint>(splitted.size());

        if (n_values == 3)
        {
            mesh._vertices(0, idx) = _getCoordinateFromObjString(splitted[0]);
            mesh._vertices(1, idx) = _getCoordinateFromObjString(splitted[1]);
            mesh._vertices(2, idx) = _getCoordinateFromObjString(splitted[2]);
        }
        else if (n_values == 4)
        {
            mesh._vertices(0, idx) = _getCoordinateFromObjString(splitted[0]);
            mesh._vertices(1, idx) = _getCoordinateFromObjString(splitted[1]);
            mesh._vertices(2, idx) = _getCoordinateFromObjString(splitted[2]);
            mesh._vertices(3, idx) = _getCoordinateFromObjString(splitted[3]);

            if (mesh._vertices(3, idx) != 1)
                mesh._is_homogenized = false;
        }
        else
        {
            std::cerr << "Error: invalid number of values (" << n_values << ") for vertex " << idx << " (" << vertices[idx] << ") in " << filename << std::endl;
            throw;
        }
    }

    // Vertex normals
    
    mesh._has_vertex_normals = n_vertex_normals > 0;

    if (mesh._has_vertex_normals)
	{
		assert(n_vertex_normals == n_vertices);
        mesh._vertex_normals = arma::Mat<imp_float>(3, n_vertices);
	}

    for (idx = 0; idx < n_vertex_normals; idx++)
    {
        splitted = string_util::split(vertex_normals[idx]);
        n_values = static_cast<imp_uint>(splitted.size());

        if (n_values == 3)
        {
            mesh._vertex_normals(0, idx) = _getCoordinateFromObjString(splitted[0]);
            mesh._vertex_normals(1, idx) = _getCoordinateFromObjString(splitted[1]);
            mesh._vertex_normals(2, idx) = _getCoordinateFromObjString(splitted[2]);
        }
        else
        {
            std::cerr << "Error: invalid number of values (" << n_values << ") for vertex normal " << idx << " (" << vertex_normals[idx] << ") in " << filename << std::endl;
            throw;
        }
    }
    
    if (mesh._has_vertex_normals)
        arma::normalise(mesh._vertex_normals);

    // Texture coordinates

	mesh._has_texture_coordinates = n_texture_coords > 0;

    if (mesh._has_texture_coordinates)
	{
		mesh._texture_coordinates.reserve(n_texture_coords);
	}

    for (idx = 0; idx < n_texture_coords; idx++)
    {
        splitted = string_util::split(texture_coords[idx]);
        n_values = static_cast<imp_uint>(splitted.size());

        if (n_values == 2 || n_values == 3)
        {
            mesh._texture_coordinates.emplace_back(_getCoordinateFromObjString(splitted[0]), _getCoordinateFromObjString(splitted[1]));
        }
		else
		{
            std::cerr << "Error: invalid number of values (" << n_values << ") for texture coordinates " << idx << " (" << texture_coords[idx] << ") in " << filename << std::endl;
            throw;
        }
    }

    // Faces

    imp_uint face_list_idx = 0;

	if (mesh._has_texture_coordinates)
	{
		mesh._faces = arma::Mat<imp_uint>(6, n_faces);

		for (idx = 0; idx < n_faces; idx++)
		{
			splitted = string_util::split(faces[idx]);
			n_values = static_cast<imp_uint>(splitted.size());

			if (n_values == 3)
			{
				mesh._faces(0, face_list_idx) = _getFaceIndexFromObjString(splitted[0], n_vertices);
				mesh._faces(1, face_list_idx) = _getFaceIndexFromObjString(splitted[1], n_vertices);
				mesh._faces(2, face_list_idx) = _getFaceIndexFromObjString(splitted[2], n_vertices);

				mesh._faces(3, face_list_idx) = _getFaceTextureIndexFromObjString(splitted[0], n_texture_coords);
				mesh._faces(4, face_list_idx) = _getFaceTextureIndexFromObjString(splitted[1], n_texture_coords);
				mesh._faces(5, face_list_idx) = _getFaceTextureIndexFromObjString(splitted[2], n_texture_coords);

				face_list_idx++;
			}
			else if (n_values == 4)
			{
				mesh._faces(0, face_list_idx) = _getFaceIndexFromObjString(splitted[0], n_vertices);
				mesh._faces(1, face_list_idx) = _getFaceIndexFromObjString(splitted[1], n_vertices);
				mesh._faces(2, face_list_idx) = _getFaceIndexFromObjString(splitted[2], n_vertices);

				mesh._faces(3, face_list_idx) = _getFaceTextureIndexFromObjString(splitted[0], n_texture_coords);
				mesh._faces(4, face_list_idx) = _getFaceTextureIndexFromObjString(splitted[1], n_texture_coords);
				mesh._faces(5, face_list_idx) = _getFaceTextureIndexFromObjString(splitted[2], n_texture_coords);

				extra_face(0) = mesh._faces(2, face_list_idx);
				extra_face(1) = _getFaceIndexFromObjString(splitted[3], n_vertices);
				extra_face(2) = mesh._faces(0, face_list_idx);

				extra_face(3) = mesh._faces(5, face_list_idx);
				extra_face(4) = _getFaceTextureIndexFromObjString(splitted[3], n_texture_coords);
				extra_face(5) = mesh._faces(3, face_list_idx);

				mesh._faces.insert_cols(++face_list_idx, extra_face);

				face_list_idx++;
			}
			else
			{
				std::cerr << "Error: invalid number of values (" << n_values << ") for face " << idx << " (" << faces[idx] << ") in " << filename << std::endl;
				throw;
			}
		}
	}
	else
	{
		mesh._faces = arma::Mat<imp_uint>(3, n_faces);

		for (idx = 0; idx < n_faces; idx++)
		{
			splitted = string_util::split(faces[idx]);
			n_values = static_cast<imp_uint>(splitted.size());

			if (n_values == 3)
			{
				mesh._faces(0, face_list_idx) = _getFaceIndexFromObjString(splitted[0], n_vertices);
				mesh._faces(1, face_list_idx) = _getFaceIndexFromObjString(splitted[1], n_vertices);
				mesh._faces(2, face_list_idx) = _getFaceIndexFromObjString(splitted[2], n_vertices);

				face_list_idx++;
			}
			else if (n_values == 4)
			{
				mesh._faces(0, face_list_idx) = _getFaceIndexFromObjString(splitted[0], n_vertices);
				mesh._faces(1, face_list_idx) = _getFaceIndexFromObjString(splitted[1], n_vertices);
				mesh._faces(2, face_list_idx) = _getFaceIndexFromObjString(splitted[2], n_vertices);

				extra_face(0) = mesh._faces(2, face_list_idx);
				extra_face(1) = _getFaceIndexFromObjString(splitted[3], n_vertices);
				extra_face(2) = mesh._faces(0, face_list_idx);

				mesh._faces.insert_cols(++face_list_idx, extra_face.subvec(0, 2));

				face_list_idx++;
			}
			else
			{
				std::cerr << "Error: invalid number of values (" << n_values << ") for face " << idx << " (" << faces[idx] << ") in " << filename << std::endl;
				throw;
			}
		}
	}

	mesh.computeFaceNormals();

	if (!mesh._has_vertex_normals)
		mesh.computeVertexNormals();

	mesh.computeTangentVectors();

    return mesh;
}

imp_float TriangleMesh::_getCoordinateFromObjString(const std::string& s)
{
    return static_cast<imp_float>(atof(s.c_str()));
}

imp_uint TriangleMesh::_getFaceIndexFromObjString(const std::string& s, imp_uint n_vertices)
{
    int face_index = atoi(string_util::split(s, '/')[0].c_str());

    if (face_index < 0)
        face_index = static_cast<imp_int>(n_vertices) - face_index;
    else
        face_index--;

    assert(face_index >= 0 && face_index < static_cast<imp_int>(n_vertices));

    return static_cast<imp_uint>(face_index);
}

imp_uint TriangleMesh::_getFaceTextureIndexFromObjString(const std::string& s, imp_uint n_texture_coordinates)
{
    int face_index = atoi(string_util::split(s, '/')[1].c_str());

    if (face_index < 0)
        face_index = static_cast<imp_int>(n_texture_coordinates) - face_index;
    else
        face_index--;

    assert(face_index >= 0 && face_index < static_cast<imp_int>(n_texture_coordinates));

    return static_cast<imp_uint>(face_index);
}

void TriangleMesh::_getObjFileData(const std::string& filename,
                                   string_vec& vertices,
                                   string_vec& texture_coords,
                                   string_vec& normals,
                                   string_vec& param_space_vertices,
                                   string_vec& faces,
                                   string_vec& material_files,
                                   string_vec& material_names,
                                   int_vec& face_material_indices)
{
    imp_uint length = static_cast<imp_uint>(filename.length());
    assert(length > 4 &&
           filename.substr(length-4, 4) == ".obj");

    std::ifstream infile(filename);
    assert(infile);

    vertices.clear();
    texture_coords.clear();
    normals.clear();
    param_space_vertices.clear();
    faces.clear();
    material_files.clear();
    material_names.clear();
    face_material_indices.clear();

    imp_uint n_lines = 0;

    std::string line, word, current_group = "";
    std::map<std::string, imp_int> group_material_indices;
    imp_int current_material_idx = -1;
    imp_uint split;
    
    group_material_indices.insert(std::make_pair(current_group, current_material_idx));

    while (std::getline(infile, line))
    {
        n_lines++;

        string_util::trim(line);

        split = static_cast<imp_uint>(line.find(" "));

        if (split == static_cast<imp_uint>(std::string::npos))
            continue;

        word = line.substr(0, split);

        if (word == "#")
        {
            continue;
        }
        else if (word == "v")
        {
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            assert(line.length() > 0);
            vertices.push_back(line);
        }
        else if (word == "vt")
        {
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            assert(line.length() > 0);
            texture_coords.push_back(line);
        }
        else if (word == "vn")
        {
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            assert(line.length() > 0);
            normals.push_back(line);
        }
        else if (word == "vp")
        {
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            assert(line.length() > 0);
            param_space_vertices.push_back(line);
        }
        else if (word == "f")
        {
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);

            assert(line.length() > 0);

            faces.push_back(line);
            face_material_indices.push_back(group_material_indices[current_group]);
        }
        else if (word == "mtllib")
        {
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            
            length = static_cast<imp_uint>(line.length());
            
            assert(length > 4 &&
                   line.substr(length-4, 4) == ".mtl");

            if (std::find(material_files.begin(), material_files.end(), line) == material_files.end())
                material_files.push_back(line);
        }
        else if (word == "g")
        {
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            assert(line.length() > 0);

            current_group = line;

            if (group_material_indices.find(current_group) == group_material_indices.end())
                group_material_indices.insert(std::make_pair(current_group, -1));
        }
        else if (word == "usemtl")
        {
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            assert(line.length() > 0);
            
            if (std::find(material_names.begin(), material_names.end(), line) == material_names.end())
            {
                material_names.push_back(line);
                current_material_idx++;
            }
            
            group_material_indices[current_group] = current_material_idx;
        }
        else
        {
            std::cerr << "Error: invalid data label (" << word << ") in line " << n_lines << " of " << filename << std::endl;
            throw;
        }
    }

    infile.close();
}

TriangleMesh TriangleMesh::triangle(const Triangle& triangle_obj)
{
    TriangleMesh triangle_mesh;

    const Point& A = triangle_obj.getPointA();
    const Point& B = triangle_obj.getPointB();
    const Point& C = triangle_obj.getPointC();

    triangle_mesh._vertices = {{A.x, B.x, C.x},
                               {A.y, B.y, C.y},
                               {A.z, B.z, C.z},
                               {1, 1, 1}};

    triangle_mesh._faces = {0, 1, 2};

	triangle_mesh.computeFaceNormals();
	triangle_mesh.computeVertexNormals();

    return triangle_mesh;
}

TriangleMesh TriangleMesh::box(const Box& box_obj)
{
    TriangleMesh box_mesh;

    const std::vector<Point>& corners = box_obj.getCorners();

    imp_float x0 = corners[0].x, y0 = corners[0].y, z0 = corners[0].z;
    imp_float x1 = corners[1].x, y1 = corners[1].y, z1 = corners[1].z;
    imp_float x2 = corners[2].x, y2 = corners[2].y, z2 = corners[2].z;
    imp_float x3 = corners[3].x, y3 = corners[3].y, z3 = corners[3].z;
    imp_float x4 = corners[4].x, y4 = corners[4].y, z4 = corners[4].z;
    imp_float x5 = corners[5].x, y5 = corners[5].y, z5 = corners[5].z;
    imp_float x6 = corners[6].x, y6 = corners[6].y, z6 = corners[6].z;
    imp_float x7 = corners[7].x, y7 = corners[7].y, z7 = corners[7].z;

    box_mesh._vertices = {{x0, x0, x0, x1, x1, x1, x2, x2, x2, x3, x3, x3, x4, x4, x4, x5, x5, x5, x6, x6, x6, x7, x7, x7},
                          {y0, y0, y0, y1, y1, y1, y2, y2, y2, y3, y3, y3, y4, y4, y4, y5, y5, y5, y6, y6, y6, y7, y7, y7},
                          {z0, z0, z0, z1, z1, z1, z2, z2, z2, z3, z3, z3, z4, z4, z4, z5, z5, z5, z6, z6, z6, z7, z7, z7},
                          { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1}};

    box_mesh._faces = {{0, 3, 12, 21,  1, 13, 19, 22,  2, 11,  5, 17},
                       {9, 9, 15, 15,  4,  4,  7,  7, 14, 14,  8,  8},
                       {3, 6, 21, 18, 13, 16, 22, 10, 11, 23, 17, 20}};
	
	box_mesh.computeFaceNormals();
	box_mesh.computeVertexNormals();

    return box_mesh;
}

TriangleMesh TriangleMesh::room(const Box& box_obj)
{
    TriangleMesh room_mesh;

    const std::vector<Point>& corners = box_obj.getCorners();

    imp_float x0 = corners[0].x, y0 = corners[0].y, z0 = corners[0].z;
    imp_float x1 = corners[1].x, y1 = corners[1].y, z1 = corners[1].z;
    imp_float x2 = corners[2].x, y2 = corners[2].y, z2 = corners[2].z;
    imp_float x3 = corners[3].x, y3 = corners[3].y, z3 = corners[3].z;
    imp_float x4 = corners[4].x, y4 = corners[4].y, z4 = corners[4].z;
    imp_float x5 = corners[5].x, y5 = corners[5].y, z5 = corners[5].z;
    imp_float x6 = corners[6].x, y6 = corners[6].y, z6 = corners[6].z;
    imp_float x7 = corners[7].x, y7 = corners[7].y, z7 = corners[7].z;

    room_mesh._vertices = {{x0, x0, x0, x1, x1, x1, x2, x2, x2, x3, x3, x3, x4, x4, x4, x5, x5, x5, x6, x6, x6, x7, x7, x7},
                           {y0, y0, y0, y1, y1, y1, y2, y2, y2, y3, y3, y3, y4, y4, y4, y5, y5, y5, y6, y6, y6, y7, y7, y7},
                           {z0, z0, z0, z1, z1, z1, z2, z2, z2, z3, z3, z3, z4, z4, z4, z5, z5, z5, z6, z6, z6, z7, z7, z7},
                           { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1}};

    room_mesh._faces = {{0, 3, 12, 21,  1, 13, 19, 22,  2, 11,  5, 17},
                        {3, 6, 21, 18, 13, 16, 22, 10, 11, 23, 17, 20},
                        {9, 9, 15, 15,  4,  4,  7,  7, 14, 14,  8,  8}};
	
	room_mesh.computeFaceNormals();
	room_mesh.computeVertexNormals();

    return room_mesh;
}

TriangleMesh TriangleMesh::sheet(const Point& center, const Vector& normal, const Vector& width_vector, imp_float height)
{
    TriangleMesh sheet_mesh;

	const Vector& nnormal = normal.getNormalized();
	const Vector& height_vector = nnormal.getUnitNormalWith(width_vector)*height;
	const Point& origin = center - width_vector*0.5f - height_vector*0.5f;

    const Point& corner_1 = origin;
    const Point& corner_2 = corner_1 + width_vector;
    const Point& corner_3 = corner_2 + height_vector;
    const Point& corner_4 = corner_1 + height_vector;

    sheet_mesh._vertices = {{corner_1.x, corner_2.x, corner_3.x, corner_4.x},
                            {corner_1.y, corner_2.y, corner_3.y, corner_4.y},
                            {corner_1.z, corner_2.z, corner_3.z, corner_4.z},
                            {         1,          1,          1,          1}};

    sheet_mesh._faces = {{1, 0},
                         {2, 2},
                         {0, 3}};

	sheet_mesh._face_normals = {{nnormal.x, nnormal.x},
								{nnormal.y, nnormal.y},
								{nnormal.z, nnormal.z}};

	sheet_mesh._vertex_normals = {{nnormal.x, nnormal.x, nnormal.x, nnormal.x},
								  {nnormal.y, nnormal.y, nnormal.y, nnormal.y},
								  {nnormal.z, nnormal.z, nnormal.z, nnormal.z}};
	
	sheet_mesh._has_face_normals = true;
	sheet_mesh._has_vertex_normals = true;

	sheet_mesh._texture_coordinates = {Point2(0, 0), Point2(1, 0), Point2(1, 1), Point2(0, 1)};

    sheet_mesh._faces = arma::join_cols(sheet_mesh._faces, sheet_mesh._faces);

	sheet_mesh._has_texture_coordinates = true;

	sheet_mesh.computeTangentVectors();

    return sheet_mesh;
}

TriangleMesh TriangleMesh::twoSidedSheet(const Point& center, const Vector& normal, const Vector& width_vector, imp_float height)
{
    TriangleMesh sheet_mesh;

	const Vector& nnormal = normal.getNormalized();
	const Vector& height_vector = nnormal.getUnitNormalWith(width_vector)*height;
	const Point& origin = center - width_vector*0.5f - height_vector*0.5f;

    const Point& corner_1 = origin;
    const Point& corner_2 = corner_1 + width_vector;
    const Point& corner_3 = corner_2 + height_vector;
    const Point& corner_4 = corner_1 + height_vector;

    sheet_mesh._vertices = {{corner_1.x, corner_2.x, corner_3.x, corner_4.x},
                            {corner_1.y, corner_2.y, corner_3.y, corner_4.y},
                            {corner_1.z, corner_2.z, corner_3.z, corner_4.z},
                            {         1,          1,          1,          1}};

    sheet_mesh._faces = {{1, 0, 1, 2},
                         {2, 2, 0, 0},
                         {0, 3, 2, 3}};

	sheet_mesh._face_normals = {{nnormal.x, nnormal.x, nnormal.x, nnormal.x},
								{nnormal.y, nnormal.y, nnormal.y, nnormal.y},
								{nnormal.z, nnormal.z, nnormal.z, nnormal.z}};

	sheet_mesh._vertex_normals = {{nnormal.x, nnormal.x, nnormal.x, nnormal.x},
								  {nnormal.y, nnormal.y, nnormal.y, nnormal.y},
								  {nnormal.z, nnormal.z, nnormal.z, nnormal.z}};
	
	sheet_mesh._has_face_normals = true;
	sheet_mesh._has_vertex_normals = true;

    return sheet_mesh;
}

TriangleMesh TriangleMesh::sphere(const Sphere& sphere_obj, imp_uint resolution)
{
    assert(resolution >= 1);

    TriangleMesh sphere_mesh;

    imp_float x0 = sphere_obj.center.x;
    imp_float y0 = sphere_obj.center.y;
    imp_float z0 = sphere_obj.center.z;
    imp_float r = sphere_obj.radius;
    
    imp_uint n_lat = resolution + 2;
    imp_uint n_lon = 2*n_lat;

    imp_uint n_vertices = 2 + (n_lat - 2)*n_lon;
    imp_uint n_faces = 2*(n_lat - 2)*n_lon;
    imp_uint i, j, n, m;

    imp_float dtheta = IMP_PI/(static_cast<imp_float>(n_lat) - 1);
    imp_float dphi = 2*IMP_PI/static_cast<imp_float>(n_lon);

    imp_float theta, phi;
    imp_float sin_theta;
    imp_uint offset, offset_prev;
    imp_uint current, right, above, above_right;
    
    sphere_mesh._vertex_normals = arma::Mat<imp_float>(3, n_vertices);

    sphere_mesh._vertices = arma::Mat<imp_float>(4, n_vertices);
    sphere_mesh._vertices.row(3).ones();

    // Top vertex
    sphere_mesh._vertex_normals(0, 0) = 0;
    sphere_mesh._vertex_normals(1, 0) = 0;
    sphere_mesh._vertex_normals(2, 0) = 1;
    sphere_mesh._vertices(0, 0) = x0;
    sphere_mesh._vertices(1, 0) = y0;
    sphere_mesh._vertices(2, 0) = z0 + r;
	sphere_mesh._texture_coordinates.emplace_back(0.0f, 1.0f);

    n = 1;
    for (i = 1; i < n_lat-1; i++) {
        for (j = 0; j < n_lon; j++)
        {
            theta = i*dtheta;
            phi = j*dphi;
            sin_theta = sin(theta);

            sphere_mesh._vertex_normals(0, n) = sin_theta*cos(phi);
            sphere_mesh._vertex_normals(1, n) = sin_theta*sin(phi);
            sphere_mesh._vertex_normals(2, n) = cos(theta);

            sphere_mesh._vertices(0, n) = x0 + r*sphere_mesh._vertex_normals(0, n);
            sphere_mesh._vertices(1, n) = y0 + r*sphere_mesh._vertex_normals(1, n);
            sphere_mesh._vertices(2, n) = z0 + r*sphere_mesh._vertex_normals(2, n);

			sphere_mesh._texture_coordinates.emplace_back(phi/IMP_TWO_PI, sphere_mesh._vertex_normals(2, n)*0.5f + 0.5f);

            n++;
        }
    }
    
    // Bottom vertex
    sphere_mesh._vertex_normals(0, n) = 0;
    sphere_mesh._vertex_normals(1, n) = 0;
    sphere_mesh._vertex_normals(2, n) = -1;
    sphere_mesh._vertices(0, n) = x0;
    sphere_mesh._vertices(1, n) = y0;
    sphere_mesh._vertices(2, n) = z0 - r;
	sphere_mesh._texture_coordinates.emplace_back(0.0f, 0.0f);

	// Add additional texture coordinates for u = 1 to ensure correct interpolation from u <~ 1 to u = 0
    sphere_mesh._texture_coordinates.emplace_back(1.0f, 1.0f);

    for (i = 1; i < n_lat-1; i++)
        sphere_mesh._texture_coordinates.emplace_back(1.0f, sphere_mesh._vertex_normals(2, 1 + (i-1)*n_lon)*0.5f + 0.5f);

    sphere_mesh._texture_coordinates.emplace_back(1.0f, 0.0f);

    sphere_mesh._faces = arma::Mat<imp_uint>(6, n_faces);

	m = n+1;
    
    // Top cone
    n = 0;
    for (j = 1; j < n_lon; j++)
    {
        sphere_mesh._faces(0, n) = j;
        sphere_mesh._faces(1, n) = j + 1;
        sphere_mesh._faces(2, n) = 0;
		
        sphere_mesh._faces(3, n) = j;
        sphere_mesh._faces(4, n) = j + 1;
        sphere_mesh._faces(5, n) = 0;

        n++;
    }
    sphere_mesh._faces(0, n) = n_lon;
    sphere_mesh._faces(1, n) = 1;
    sphere_mesh._faces(2, n) = 0;
		
    sphere_mesh._faces(3, n) = n_lon;
    sphere_mesh._faces(4, n) = m + 1;
    sphere_mesh._faces(5, n) = m;

    n++;
	m++;
    
    for (i = 1; i < n_lat-2; i++)
    {
        offset = 1 + i*n_lon;
        offset_prev = offset - n_lon;

        for (j = 0; j < n_lon-1; j++)
        {
            current = offset + j;
            right = current + 1;
            above = offset_prev + j;
            above_right = above + 1;

            sphere_mesh._faces(0, n) = current;
            sphere_mesh._faces(1, n) = right;
            sphere_mesh._faces(2, n) = above_right;

            sphere_mesh._faces(3, n) = current;
            sphere_mesh._faces(4, n) = right;
            sphere_mesh._faces(5, n) = above_right;

            n++;

            sphere_mesh._faces(0, n) = above_right;
            sphere_mesh._faces(1, n) = above;
            sphere_mesh._faces(2, n) = current;

            sphere_mesh._faces(3, n) = above_right;
            sphere_mesh._faces(4, n) = above;
            sphere_mesh._faces(5, n) = current;

            n++;
        }
        
        j = n_lon - 1;

        current =     offset + j;
        right =       offset;
        above =       offset_prev + j;
        above_right = offset_prev;

        sphere_mesh._faces(0, n) = current;
        sphere_mesh._faces(1, n) = right;
        sphere_mesh._faces(2, n) = above_right;

        sphere_mesh._faces(3, n) = current;
        sphere_mesh._faces(4, n) = m + 1;
        sphere_mesh._faces(5, n) = m;

        n++;

        sphere_mesh._faces(0, n) = above_right;
        sphere_mesh._faces(1, n) = above;
        sphere_mesh._faces(2, n) = current;

        sphere_mesh._faces(3, n) = m;
        sphere_mesh._faces(4, n) = above;
        sphere_mesh._faces(5, n) = current;

        n++;
		m++;
    }

    // Bottom cone
    offset = 1 + (n_lat - 3)*n_lon;
    for (j = 0; j < n_lon-1; j++)
    {
        sphere_mesh._faces(0, n) = offset + j + 1;
        sphere_mesh._faces(1, n) = offset + j;
        sphere_mesh._faces(2, n) = n_vertices - 1;

        sphere_mesh._faces(3, n) = offset + j + 1;
        sphere_mesh._faces(4, n) = offset + j;
        sphere_mesh._faces(5, n) = n_vertices - 1;

        n++;
    }
    sphere_mesh._faces(0, n) = offset;
    sphere_mesh._faces(1, n) = offset + n_lon - 1;
    sphere_mesh._faces(2, n) = n_vertices - 1;

    sphere_mesh._faces(3, n) = offset;
    sphere_mesh._faces(4, n) = m;
    sphere_mesh._faces(5, n) = m + 1;

	sphere_mesh._has_vertex_normals = true;
	sphere_mesh._has_texture_coordinates = true;

	sphere_mesh.computeFaceNormals();

	sphere_mesh.computeTangentVectors();

    return sphere_mesh;
}

TriangleMesh TriangleMesh::twoSidedSphere(const Sphere& sphere_obj, imp_uint resolution)
{
    assert(resolution >= 1);

    TriangleMesh sphere_mesh;

    imp_float x0 = sphere_obj.center.x;
    imp_float y0 = sphere_obj.center.y;
    imp_float z0 = sphere_obj.center.z;
    imp_float r = sphere_obj.radius;
    
    imp_uint n_lat = resolution + 2;
    imp_uint n_lon = 2*n_lat;

    imp_uint n_vertices = 2 + (n_lat - 2)*n_lon;
    imp_uint n_faces = 2*(n_lat - 2)*n_lon;
    imp_uint i, j, n;

    imp_float dtheta = IMP_PI/(static_cast<imp_float>(n_lat) - 1);
    imp_float dphi = 2*IMP_PI/static_cast<imp_float>(n_lon);

    imp_float theta, phi;
    imp_float sin_theta;
    imp_uint offset, offset_prev;
    imp_uint current, right, above, above_right;
    
    sphere_mesh._vertex_normals = arma::Mat<imp_float>(3, n_vertices);
    sphere_mesh._vertices = arma::Mat<imp_float>(4, n_vertices);
    sphere_mesh._vertices.row(3).ones();

    // Top vertex
    sphere_mesh._vertex_normals(0, 0) = 0;
    sphere_mesh._vertex_normals(1, 0) = 0;
    sphere_mesh._vertex_normals(2, 0) = 1;
    sphere_mesh._vertices(0, 0) = x0;
    sphere_mesh._vertices(1, 0) = y0;
    sphere_mesh._vertices(2, 0) = z0 + r;

    n = 1;
    for (i = 1; i < n_lat - 1; i++) {
        for (j = 0; j < n_lon; j++)
        {
            theta = i*dtheta;
            phi = j*dphi;
            sin_theta = sin(theta);

            sphere_mesh._vertex_normals(0, n) = sin_theta*cos(phi);
            sphere_mesh._vertex_normals(1, n) = sin_theta*sin(phi);
            sphere_mesh._vertex_normals(2, n) = cos(theta);

            sphere_mesh._vertices(0, n) = x0 + r*sphere_mesh._vertex_normals(0, n);
            sphere_mesh._vertices(1, n) = y0 + r*sphere_mesh._vertex_normals(1, n);
            sphere_mesh._vertices(2, n) = z0 + r*sphere_mesh._vertex_normals(2, n);

            n++;
        }
    }
    
    // Bottom vertex
    sphere_mesh._vertex_normals(0, n) = 0;
    sphere_mesh._vertex_normals(1, n) = 0;
    sphere_mesh._vertex_normals(2, n) = -1;
    sphere_mesh._vertices(0, n) = x0;
    sphere_mesh._vertices(1, n) = y0;
    sphere_mesh._vertices(2, n) = z0 - r;

    sphere_mesh._faces = arma::Mat<imp_uint>(3, 2*n_faces);

	sphere_mesh._has_vertex_normals = true;
    
    // Top cone
    n = 0;
    for (j = 1; j < n_lon; j++)
    {
        sphere_mesh._faces(0, n) = j;
        sphere_mesh._faces(1, n) = j + 1;
        sphere_mesh._faces(2, n) = 0;

        sphere_mesh._faces(1, n_faces+n) = j;
        sphere_mesh._faces(0, n_faces+n) = j + 1;
        sphere_mesh._faces(2, n_faces+n) = 0;
        n++;
    }
    sphere_mesh._faces(0, n) = n_lon;
    sphere_mesh._faces(1, n) = 1;
    sphere_mesh._faces(2, n) = 0;
	
    sphere_mesh._faces(1, n_faces+n) = n_lon;
    sphere_mesh._faces(0, n_faces+n) = 1;
    sphere_mesh._faces(2, n_faces+n) = 0;
    n++;
    
    for (i = 1; i < n_lat-2; i++)
    {
        offset = 1 + i*n_lon;
        offset_prev = offset - n_lon;

        for (j = 0; j < n_lon-1; j++)
        {
            current = offset + j;
            right = current + 1;
            above = offset_prev + j;
            above_right = above + 1;

            sphere_mesh._faces(0, n) = current;
            sphere_mesh._faces(1, n) = right;
            sphere_mesh._faces(2, n) = above_right;
			
            sphere_mesh._faces(1, n_faces+n) = current;
            sphere_mesh._faces(0, n_faces+n) = right;
            sphere_mesh._faces(2, n_faces+n) = above_right;
            n++;

            sphere_mesh._faces(0, n) = above_right;
            sphere_mesh._faces(1, n) = above;
            sphere_mesh._faces(2, n) = current;
			
            sphere_mesh._faces(1, n_faces+n) = above_right;
            sphere_mesh._faces(0, n_faces+n) = above;
            sphere_mesh._faces(2, n_faces+n) = current;
            n++;
        }
        
        j = n_lon - 1;

        current =     offset + j;
        right =       offset;
        above =       offset_prev + j;
        above_right = offset_prev;

        sphere_mesh._faces(0, n) = current;
        sphere_mesh._faces(1, n) = right;
        sphere_mesh._faces(2, n) = above_right;
		
        sphere_mesh._faces(1, n_faces+n) = current;
        sphere_mesh._faces(0, n_faces+n) = right;
        sphere_mesh._faces(2, n_faces+n) = above_right;
        n++;

        sphere_mesh._faces(0, n) = above_right;
        sphere_mesh._faces(1, n) = above;
        sphere_mesh._faces(2, n) = current;

        sphere_mesh._faces(1, n_faces+n) = above_right;
        sphere_mesh._faces(0, n_faces+n) = above;
        sphere_mesh._faces(2, n_faces+n) = current;
        n++;
    }

    // Bottom cone
    offset = 1 + (n_lat - 3)*n_lon;
    for (j = 0; j < n_lon-1; j++)
    {
        sphere_mesh._faces(0, n) = offset + j + 1;
        sphere_mesh._faces(1, n) = offset + j;
        sphere_mesh._faces(2, n) = n_vertices - 1;
		
        sphere_mesh._faces(1, n_faces+n) = offset + j + 1;
        sphere_mesh._faces(0, n_faces+n) = offset + j;
        sphere_mesh._faces(2, n_faces+n) = n_vertices - 1;
        n++;
    }
    sphere_mesh._faces(0, n) = offset;
    sphere_mesh._faces(1, n) = offset + n_lon - 1;
    sphere_mesh._faces(2, n) = n_vertices - 1;
	
    sphere_mesh._faces(1, n_faces+n) = offset;
    sphere_mesh._faces(0, n_faces+n) = offset + n_lon - 1;
    sphere_mesh._faces(2, n_faces+n) = n_vertices - 1;

	sphere_mesh.computeFaceNormals();

    return sphere_mesh;
}

void TriangleMesh::initializeVertexData3()
{
	if (_has_vertex_data_3)
		return;

	_vertex_data_3 = arma::Mat<imp_float>(3, getNumberOfVertices(), arma::fill::zeros);
	_has_vertex_data_3 = true;
}

void TriangleMesh::setVertexData3(imp_uint idx,
								  imp_float data_0,
								  imp_float data_1,
								  imp_float data_2)
{
	assert(_has_vertex_data_3);

	_vertex_data_3(0, idx) = data_0;
	_vertex_data_3(1, idx) = data_1;
	_vertex_data_3(2, idx) = data_2;
}

void TriangleMesh::addVertex(imp_float x, imp_float y, imp_float z)
{
    _vertices.insert_cols(_vertices.n_cols, arma::Col<imp_float>({x, y, z, 1}));

    _has_vertex_normals = false;
    _has_vertex_tangents = false;
    _has_aabb = false;
	_has_vertex_data_3 = false;
}

void TriangleMesh::addVertex(const Point& vertex)
{
    addVertex(vertex.x, vertex.y, vertex.z);
}

void TriangleMesh::addFace(imp_uint i, imp_uint j, imp_uint k)
{
	assert(!_has_texture_coordinates);

    _faces.insert_cols(_faces.n_cols, arma::Col<imp_uint>({i, j, k}));
	
    _has_face_normals = false;
    _has_aabb = false;
}

void TriangleMesh::addFace(imp_uint i, imp_uint j, imp_uint k,
						   imp_uint l, imp_uint m, imp_uint n)
{
	assert(_has_texture_coordinates);

    _faces.insert_cols(_faces.n_cols, arma::Col<imp_uint>({i, j, k, l, m, n}));
	
    _has_face_normals = false;
    _has_aabb = false;
}

void TriangleMesh::removeVertex(imp_uint idx)
{
    arma::uvec occurences = arma::unique(arma::find(_faces == idx)/((_has_texture_coordinates)? 6 : 3));
    arma::uvec::const_iterator iter = occurences.end();
    while (iter != occurences.begin())
    {
        --iter;

        _faces.shed_col(*iter);

		if (_has_face_normals)
			_face_normals.shed_col(*iter);
    }

    _vertices.shed_col(idx);

	if (_has_vertex_normals)
		_vertex_normals.shed_col(idx);

	if (_has_vertex_tangents)
		_vertex_tangents.shed_col(idx);

	if (_has_vertex_data_3)
		_vertex_data_3.shed_col(idx);

    occurences = arma::find(_faces > idx);
    for (iter = occurences.begin(); iter != occurences.end(); ++iter)
    {
        _faces(*iter) += 1;
    }
	
    _has_aabb = false;
}

void TriangleMesh::removeFace(imp_uint idx)
{
    _faces.shed_col(idx);
	
	if (_has_face_normals)
		_face_normals.shed_col(idx);

    _has_aabb = false;
}

void TriangleMesh::splitFaces(imp_uint n_times)
{
    imp_uint n_faces;
    imp_uint idx, i, j, k;
    Point A, B, C;
    imp_float AB_length, AC_length, BC_length;
    bool AB_exceeds_AC, AB_exceeds_BC, AC_exceeds_BC;
    imp_uint last_vertex_idx = getNumberOfVertices() - 1;

    for (imp_uint n = 0; n < n_times; n++)
    {
        n_faces = getNumberOfFaces();

        for (idx = 0; idx < n_faces; idx++)
        {
            i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);
            A.x = _vertices(0, i); A.y = _vertices(1, i); A.z = _vertices(2, i);
            B.x = _vertices(0, j); B.y = _vertices(1, j); B.z = _vertices(2, j);
            C.x = _vertices(0, k); C.y = _vertices(1, k); C.z = _vertices(2, k);

            const Vector& AB = B - A;
            const Vector& AC = C - A;
            const Vector& BC = C - B;

            AB_length = AB.getLength();
            AC_length = AC.getLength();
            BC_length = BC.getLength();

            AB_exceeds_AC = AB_length > AC_length;
            AB_exceeds_BC = AB_length > BC_length;
            AC_exceeds_BC = AC_length > BC_length;

            if (AB_exceeds_AC && AB_exceeds_BC)
            {
                addVertex(A + AB*0.5);
                last_vertex_idx++;
                _faces(1, idx) = last_vertex_idx;
                addFace(last_vertex_idx, j, k);
            }
            else if (AC_exceeds_BC)
            {
                addVertex(A + AC*0.5);
                last_vertex_idx++;
                _faces(2, idx) = last_vertex_idx;
                addFace(last_vertex_idx, j, k);
            }
            else
            {
                addVertex(B + BC*0.5);
                last_vertex_idx++;
                _faces(2, idx) = last_vertex_idx;
                addFace(last_vertex_idx, k, i);
            }
        }
    }
}

void TriangleMesh::clipNearPlaneAt(imp_float z_near)
{
    assert(_is_homogenized);
    _clip(2, z_near, 1); // Clip away points with z > z_near
}

void TriangleMesh::clipLeftPlane()
{
    assert(_is_homogenized);
    _clip(0, -1, -1); // Clip away points with x < -1
}

void TriangleMesh::clipRightPlane()
{
    assert(_is_homogenized);
    _clip(0, 1, 1); // Clip away points with x > 1
}

void TriangleMesh::clipBottomPlane()
{
    assert(_is_homogenized);
    _clip(1, -1, -1); // Clip away points with y < -1
}

void TriangleMesh::clipTopPlane()
{
    assert(_is_homogenized);
    _clip(1, 1, 1); // Clip away points with y > 1
}

void TriangleMesh::clipNearPlane()
{
    _clip(2, 0, 1); // Clip away points with z > 0
}

void TriangleMesh::clipFarPlane()
{
    assert(_is_homogenized);
    _clip(2, -1, -1); // Clip away points with z < -1
}

void TriangleMesh::clipNonNearPlanes()
{
	assert(_has_aabb);

    const Point& lower_corner = _aabb.lower_corner;
    const Point& upper_corner = _aabb.upper_corner;

    if (lower_corner.x < -1) clipLeftPlane();
    if (upper_corner.x >  1) clipRightPlane();
    if (lower_corner.y < -1) clipBottomPlane();
    if (upper_corner.y >  1) clipTopPlane();
    if (lower_corner.z < -1) clipFarPlane();
}

void TriangleMesh::_clip(imp_uint component, imp_float limit, int sign)
{
    assert(component < 3);
    assert(sign == 1 || sign == -1);

    imp_uint n_vertices = getNumberOfVertices();

    imp_uint component_1 = (component + 1) % 3;
    imp_uint component_2 = (component + 2) % 3;

    imp_float signed_limit = sign*limit;

    imp_uint i, j, k;
    imp_uint l, m, n;
    imp_float A[4], B[4], C[4];

    bool A_outside, B_outside, C_outside;

	bool was_added;

    imp_uint n_outside;
    imp_uint inside_idx_1, inside_idx_2;
    imp_uint texture_inside_idx_1, texture_inside_idx_2;

    imp_uint n_faces = getNumberOfFaces();
    imp_uint last_vertex = n_vertices - 1;
    imp_uint last_texture_coord = getNumberOfTextureCoordinates() - 1;
    
    std::vector<imp_uint> deleted_faces;

    for (imp_uint idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

		if (_has_texture_coordinates)
		{
			l = _faces(3, idx); m = _faces(4, idx); n = _faces(5, idx);
		}
        
        A[0] = _vertices(0, i); A[1] = _vertices(1, i); A[2] = _vertices(2, i); A[3] = _vertices(3, i);
        B[0] = _vertices(0, j); B[1] = _vertices(1, j); B[2] = _vertices(2, j); B[3] = _vertices(3, j);
        C[0] = _vertices(0, k); C[1] = _vertices(1, k); C[2] = _vertices(2, k); C[3] = _vertices(3, k);

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
					texture_inside_idx_1 = m;
					texture_inside_idx_2 = n;
                    was_added = _addIntersectionVertices(i, j, k,
														 l, m, n,
											             A, B, C,
														 component, component_1, component_2,
														 limit);
                }
                else if (B_outside)
                {
                    inside_idx_1 = k;
                    inside_idx_2 = i;
					texture_inside_idx_1 = n;
					texture_inside_idx_2 = l;
                    was_added = _addIntersectionVertices(i, j, k,
														 l, m, n,
														 B, C, A,
														 component, component_1, component_2,
														 limit);
                }
                else
                {
                    inside_idx_1 = i;
                    inside_idx_2 = j;
					texture_inside_idx_1 = l;
					texture_inside_idx_2 = m;
                    was_added = _addIntersectionVertices(i, j, k,
														 l, m, n,
													     C, A, B,
													     component, component_1, component_2,
														 limit);
                }

				if (was_added)
				{
					_faces(0, idx) = inside_idx_1;
					_faces(1, idx) = inside_idx_2;
					_faces(2, idx) = last_vertex + 1;

					if (_has_texture_coordinates)
					{
						_faces(3, idx) = texture_inside_idx_1;
						_faces(4, idx) = texture_inside_idx_2;
						_faces(5, idx) = last_texture_coord + 1;

						_faces.insert_cols(_faces.n_cols, arma::Col<imp_uint>({last_vertex + 1, inside_idx_2, last_vertex + 2, last_texture_coord + 1, texture_inside_idx_2, last_texture_coord + 2}));
						
						last_texture_coord += 2;
					}
					else
					{
						_faces.insert_cols(_faces.n_cols, arma::Col<imp_uint>({last_vertex + 1, inside_idx_2, last_vertex + 2}));
					}

					last_vertex += 2;

					if (_has_face_normals)
						_face_normals.insert_cols(_face_normals.n_cols, _face_normals.col(idx));
				}
				else
				{
					n_outside = 3;
				}

            }
            else if (n_outside == 2)
            {
                if (!A_outside) 
                {
                    inside_idx_1 = i;
					texture_inside_idx_1 = l;
                    was_added = _addIntersectionVertices(i, j, k,
														 l, m, n,
														 A, B, C,
														 component, component_1, component_2,
														 limit);
                }
                else if (!B_outside)
                {
                    inside_idx_1 = j;
					texture_inside_idx_1 = m;
                    was_added = _addIntersectionVertices(i, j, k,
														 l, m, n,
														 B, C, A,
														 component, component_1, component_2,
														 limit);
                }
                else                
                {
                    inside_idx_1 = k;
					texture_inside_idx_1 = n;
                    was_added = _addIntersectionVertices(i, j, k,
														 l, m, n,
														 C, A, B,
													     component, component_1, component_2,
														 limit);
                }
            
				if (was_added)
				{
					_faces(0, idx) = inside_idx_1;
					_faces(1, idx) = last_vertex + 1;
					_faces(2, idx) = last_vertex + 2;

					if (_has_texture_coordinates)
					{
						_faces(3, idx) = texture_inside_idx_1;
						_faces(4, idx) = last_texture_coord + 1;
						_faces(5, idx) = last_texture_coord + 2;
                
						last_texture_coord += 2;
					}
                
					last_vertex += 2;
				}
				else
				{
					n_outside = 3;
				}

            }

            if (n_outside == 3)
            {
                deleted_faces.push_back(idx);
            }
        }
    }

    while (!deleted_faces.empty())
    {
		removeFace(deleted_faces.back());
        deleted_faces.pop_back();
    }
}

bool TriangleMesh::_addIntersectionVertices(imp_uint i, imp_uint j, imp_uint k,
											imp_uint l, imp_uint m, imp_uint n,
											imp_float origin[], imp_float other_1[], imp_float other_2[],
											imp_uint component_0, imp_uint component_1, imp_uint component_2,
											imp_float limit)
{
    // Find vertices

    arma::Col<imp_float> vertex_vector1(4);
    arma::Col<imp_float> vertex_vector2(4);

    imp_float orig_plane_dist = limit - origin[component_0];

    imp_float scaling = orig_plane_dist/(other_1[component_0] - origin[component_0]);
    vertex_vector1(component_0) = limit;
    vertex_vector1(component_1) = origin[component_1] + scaling*(other_1[component_1] - origin[component_1]);
    vertex_vector1(component_2) = origin[component_2] + scaling*(other_1[component_2] - origin[component_2]);
    vertex_vector1(3) = origin[3] + scaling*(other_1[3] - origin[3]);

    scaling = orig_plane_dist/(other_2[component_0] - origin[component_0]);
    vertex_vector2(component_0) = limit;
    vertex_vector2(component_1) = origin[component_1] + scaling*(other_2[component_1] - origin[component_1]);
    vertex_vector2(component_2) = origin[component_2] + scaling*(other_2[component_2] - origin[component_2]);
    vertex_vector2(3) = origin[3] + scaling*(other_2[3] - origin[3]);

    imp_float alpha1, beta1, gamma1;
    imp_float alpha2, beta2, gamma2;

    // Find barycentric coordinates of new vertices in old triangle
    if (_has_vertex_normals || _has_texture_coordinates || _has_vertex_tangents || _has_vertex_data_3)
    {
        Triangle face(Point(_vertices(0, i), _vertices(1, i), _vertices(2, i)),
                      Point(_vertices(0, j), _vertices(1, j), _vertices(2, j)),
                      Point(_vertices(0, k), _vertices(1, k), _vertices(2, k)));

        face.computeNormalVectors();

		if (face.isDegenerate())
			return false;
        
        Point vertex1(vertex_vector1(0), vertex_vector1(1), vertex_vector1(2));
        Point vertex2(vertex_vector2(0), vertex_vector2(1), vertex_vector2(2));

        face.getBarycentricCoordinatesInside(vertex1, alpha1, beta1, gamma1);
        face.getBarycentricCoordinatesInside(vertex2, alpha2, beta2, gamma2);
	}
	
	// Add vertex normals
	if (_has_vertex_normals)
	{
        _vertex_normals.insert_cols(_vertex_normals.n_cols, arma::normalise(alpha1*_vertex_normals.col(i) + beta1*_vertex_normals.col(j) + gamma1*_vertex_normals.col(k)));
        _vertex_normals.insert_cols(_vertex_normals.n_cols, arma::normalise(alpha2*_vertex_normals.col(i) + beta2*_vertex_normals.col(j) + gamma2*_vertex_normals.col(k)));
    }
	
	// Add vertex tangents
	if (_has_vertex_normals)
	{
		// Note: handedness is also interpolated here. Could be a problem if it not the same for all the vertices. Also, precision errors could make it different from +-1.
        _vertex_tangents.insert_cols(_vertex_tangents.n_cols, arma::normalise(alpha1*_vertex_tangents.col(i) + beta1*_vertex_tangents.col(j) + gamma1*_vertex_tangents.col(k)));
        _vertex_tangents.insert_cols(_vertex_tangents.n_cols, arma::normalise(alpha2*_vertex_tangents.col(i) + beta2*_vertex_tangents.col(j) + gamma2*_vertex_tangents.col(k)));
    }
	
	// Add texture coordinates
	if (_has_texture_coordinates)
	{
		_texture_coordinates.push_back(_texture_coordinates[l] + (_texture_coordinates[m] - _texture_coordinates[l])*beta1 + (_texture_coordinates[n] - _texture_coordinates[l])*gamma1);
		_texture_coordinates.push_back(_texture_coordinates[l] + (_texture_coordinates[m] - _texture_coordinates[l])*beta2 + (_texture_coordinates[n] - _texture_coordinates[l])*gamma2);
    }
	
	// Add 3D vertex data
	if (_has_vertex_data_3)
	{
        _vertex_data_3.insert_cols(_vertex_data_3.n_cols, alpha1*_vertex_data_3.col(i) + beta1*_vertex_data_3.col(j) + gamma1*_vertex_data_3.col(k));
        _vertex_data_3.insert_cols(_vertex_data_3.n_cols, alpha2*_vertex_data_3.col(i) + beta2*_vertex_data_3.col(j) + gamma2*_vertex_data_3.col(k));
    }
    
    _vertices.insert_cols(_vertices.n_cols, vertex_vector1);
    _vertices.insert_cols(_vertices.n_cols, vertex_vector2);

	return true;
}

void TriangleMesh::removeBackwardFacingFaces()
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

void TriangleMesh::computeAABB()
{
	if (_has_aabb)
		return;
    
    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k;
    imp_float x, y, z;
	
	assert(getNumberOfFaces() > 0);
    i = _faces(0, 0), j = _faces(1, 0), k = _faces(2, 0);
    x = _vertices(0, i), y = _vertices(1, i), z = _vertices(2, i);

    _aabb.lower_corner.moveTo(x, y, z);
    _aabb.upper_corner.moveTo(x, y, z);

    for (imp_uint face_idx = 1; face_idx < n_faces; face_idx++)
    {
        i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

        x = _vertices(0, i), y = _vertices(1, i), z = _vertices(2, i);
        if      (x < _aabb.lower_corner.x) _aabb.lower_corner.x = x;
        else if (x > _aabb.upper_corner.x) _aabb.upper_corner.x = x;
        if      (y < _aabb.lower_corner.y) _aabb.lower_corner.y = y;
        else if (y > _aabb.upper_corner.y) _aabb.upper_corner.y = y;
        if      (z < _aabb.lower_corner.z) _aabb.lower_corner.z = z;
        else if (z > _aabb.upper_corner.z) _aabb.upper_corner.z = z;

        x = _vertices(0, j), y = _vertices(1, j), z = _vertices(2, j);
        if      (x < _aabb.lower_corner.x) _aabb.lower_corner.x = x;
        else if (x > _aabb.upper_corner.x) _aabb.upper_corner.x = x;
        if      (y < _aabb.lower_corner.y) _aabb.lower_corner.y = y;
        else if (y > _aabb.upper_corner.y) _aabb.upper_corner.y = y;
        if      (z < _aabb.lower_corner.z) _aabb.lower_corner.z = z;
        else if (z > _aabb.upper_corner.z) _aabb.upper_corner.z = z;

        x = _vertices(0, k), y = _vertices(1, k), z = _vertices(2, k);
        if      (x < _aabb.lower_corner.x) _aabb.lower_corner.x = x;
        else if (x > _aabb.upper_corner.x) _aabb.upper_corner.x = x;
        if      (y < _aabb.lower_corner.y) _aabb.lower_corner.y = y;
        else if (y > _aabb.upper_corner.y) _aabb.upper_corner.y = y;
        if      (z < _aabb.lower_corner.z) _aabb.lower_corner.z = z;
        else if (z > _aabb.upper_corner.z) _aabb.upper_corner.z = z;
    }

    _has_aabb = true;
}

void TriangleMesh::computeBoundingVolumeHierarchy()
{
	assert(_has_aabb);

    imp_uint n_triangles = getNumberOfFaces();
    std::vector<AABBContainer> objects(n_triangles);

    for (imp_uint face_idx = 0; face_idx < n_triangles; face_idx++)
    {
        const Triangle& face = getFace(face_idx);
        objects[face_idx].aabb = face.getAABB();
        objects[face_idx].centroid = face.getCentroid();
        objects[face_idx].id = face_idx;
    }

    _bounding_volume_hierarchy = BoundingVolumeHierarchy(_aabb, objects);

    objects.clear();
}

void TriangleMesh::computeBoundingAreaHierarchy(imp_float image_width,
												imp_float image_height,
												imp_float inverse_image_width_at_unit_distance_from_camera,
												imp_float inverse_image_height_at_unit_distance_from_camera)
{
    imp_uint n_triangles = getNumberOfFaces();
    std::vector<AABRContainer> objects;

	objects.reserve(n_triangles);

    AxisAlignedRectangle aabr(Point2::max(), Point2::min());

    for (imp_uint face_idx = 0; face_idx < n_triangles; face_idx++)
    {
		if (!faceFacesOrigin(face_idx))
			continue;

        const Triangle2& projected_face = getProjectedFace(face_idx,
														   image_width,
														   image_height,
														   inverse_image_width_at_unit_distance_from_camera,
														   inverse_image_height_at_unit_distance_from_camera);

		objects.emplace_back(projected_face.getAABR(), projected_face.getCentroid(), face_idx);
        aabr.merge(objects.back().aabr);
    }

    _bounding_area_hierarchy = BoundingAreaHierarchy(aabr, objects);

    objects.clear();
}

void TriangleMesh::computeFaceNormals()
{
    if (_has_face_normals)
		return;
    
    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k;

    _face_normals = arma::Mat<imp_float>(3, n_faces, arma::fill::zeros);

    for (imp_uint idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        const Vector& normal = Triangle::areaVector(Point(_vertices(0, i), _vertices(1, i), _vertices(2, i)),
                                                    Point(_vertices(0, j), _vertices(1, j), _vertices(2, j)),
                                                    Point(_vertices(0, k), _vertices(1, k), _vertices(2, k)));

        _face_normals(0, idx) = normal.x;
        _face_normals(1, idx) = normal.y;
        _face_normals(2, idx) = normal.z;
    }
    
    _has_face_normals = true;
}

void TriangleMesh::computeVertexNormals()
{
    if (_has_vertex_normals)
		return;

	assert(_has_face_normals);

    _vertex_normals = arma::Mat<imp_float>(3, getNumberOfVertices(), arma::fill::zeros);
    
    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k;

    for (imp_uint idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

		_vertex_normals.col(i) += _face_normals.col(idx);
		_vertex_normals.col(j) += _face_normals.col(idx);
		_vertex_normals.col(k) += _face_normals.col(idx);
    }

	_vertex_normals = arma::normalise(_vertex_normals);
    
    _has_vertex_normals = true;
}

void TriangleMesh::computeTangentVectors()
{
    if (_has_vertex_tangents)
		return;

	assert(_has_vertex_normals);
	assert(_has_texture_coordinates);
    
    imp_uint n_vertices = getNumberOfVertices();
    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k, l, m, n;
	imp_uint idx;
	Vector face_edge_1, face_edge_2, vertex_normal;
	imp_float tangent_vector_scale;
	imp_float handedness;

    _vertex_tangents = arma::Mat<imp_float>(4, n_vertices);

	std::vector<Vector> tangents(n_vertices), bitangents(n_vertices);

    for (idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);
        l = _faces(3, idx); m = _faces(4, idx); n = _faces(5, idx);

		face_edge_1.setComponents(_vertices(0, j) - _vertices(0, i),
								  _vertices(1, j) - _vertices(1, i),
								  _vertices(2, j) - _vertices(2, i));

		face_edge_2.setComponents(_vertices(0, k) - _vertices(0, i),
								  _vertices(1, k) - _vertices(1, i),
								  _vertices(2, k) - _vertices(2, i));

		const Vector2& uv_edge_1 = _texture_coordinates[m] - _texture_coordinates[l];
		const Vector2& uv_edge_2 = _texture_coordinates[n] - _texture_coordinates[l];

		tangent_vector_scale = 1.0f/(uv_edge_1.x*uv_edge_2.y - uv_edge_2.x*uv_edge_1.y);

		// Compute tangent vectors for current face
		const Vector& tangent = (face_edge_1*uv_edge_2.y - face_edge_2*uv_edge_1.y)*tangent_vector_scale;
		const Vector& bitangent = -(face_edge_1*uv_edge_2.x - face_edge_2*uv_edge_1.x)*tangent_vector_scale;

		// Average adjacent face tangent vectors to obtain vertex tangent vectors

		tangents[i] += tangent;
		tangents[j] += tangent;
		tangents[k] += tangent;

		bitangents[i] += bitangent;
		bitangents[j] += bitangent;
		bitangents[k] += bitangent;
    }

	for (idx = 0; idx < n_vertices; idx++)
	{
		vertex_normal.setComponents(_vertex_normals(0, idx), _vertex_normals(1, idx), _vertex_normals(2, idx));

		const Vector& tangent = tangents[idx];
		const Vector& bitangent = bitangents[idx];

		const Vector& orthogonalized_tangent = (tangent - vertex_normal*(vertex_normal.dot(tangent))).getNormalized();

		handedness = (bitangent.dot(vertex_normal.cross(tangent)) < 0.0f)? -1.0f : 1.0f;

		_vertex_tangents(0, idx) = orthogonalized_tangent.x;
		_vertex_tangents(1, idx) = orthogonalized_tangent.y;
		_vertex_tangents(2, idx) = orthogonalized_tangent.z;
		_vertex_tangents(3, idx) = handedness;
	}

	tangents.clear();
	bitangents.clear();
    
    _has_vertex_tangents = true;
}

void TriangleMesh::homogenizeVertices()
{
    const arma::Row<imp_float>& norm_vals = 1/_vertices.row(3);
    _vertices.each_row(arma::uvec({0, 1, 2})) %= norm_vals;
    _vertices.row(3).ones();

    _is_homogenized = true;
    _has_vertex_normals = false;
    _has_vertex_tangents = false;
    _has_face_normals = false;
    _has_aabb = false;
}

imp_float TriangleMesh::evaluateRayIntersection(const Ray& ray, MeshIntersectionData& intersection_data) const
{
	assert(_has_aabb);
    return _bounding_volume_hierarchy.evaluateRayIntersection(*this, ray, intersection_data);
}

bool TriangleMesh::evaluateRayAABBIntersection(const Ray& ray) const
{
	assert(_has_aabb);
    return _aabb.evaluateRayIntersection(ray) < IMP_FLOAT_INF;
}

imp_float TriangleMesh::evaluateRayFaceIntersectionNonOptimized(const Ray& ray, imp_uint face_idx, imp_float& alpha, imp_float& beta, imp_float& gamma) const
{
    imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);
    Vector AB(_vertices(0, j) - _vertices(0, i), _vertices(1, j) - _vertices(1, i), _vertices(2, j) - _vertices(2, i));
    Vector AC(_vertices(0, k) - _vertices(0, i), _vertices(1, k) - _vertices(1, i), _vertices(2, k) - _vertices(2, i));

    const Vector& q = ray.direction.cross(AC);
    imp_float impact_factor = AB.dot(q); // Can be zero, resulting in infinite barycentric coordinates

    const Vector& displacement = ray.origin - Point(_vertices(0, i), _vertices(1, i), _vertices(2, i));
    const Vector& r = displacement.cross(AB);

    beta = displacement.dot(q)/impact_factor;
    gamma = ray.direction.dot(r)/impact_factor;
    alpha = 1 - beta - gamma;

    imp_float distance = AC.dot(r)/impact_factor;

    if (impact_factor <= _eps_impact_factor ||
        alpha < _eps_coordinates || beta < _eps_coordinates || gamma < _eps_coordinates ||
        distance <= 0)
    {
        distance = IMP_FLOAT_INF;
    }

    return distance;
}

imp_float TriangleMesh::evaluateRayFaceIntersection(const Ray& ray, MeshIntersectionData& intersection_data) const
{
    // intersection_data.face_id must be set to the index of the face to test

    imp_uint i = _faces(0, intersection_data.face_id), j = _faces(1, intersection_data.face_id), k = _faces(2, intersection_data.face_id);

    imp_float Ax = _vertices(0, i), Ay = _vertices(1, i), Az = _vertices(2, i);
    imp_float Bx = _vertices(0, j), By = _vertices(1, j), Bz = _vertices(2, j);
    imp_float Cx = _vertices(0, k), Cy = _vertices(1, k), Cz = _vertices(2, k);

    imp_float rox = ray.origin.x, roy = ray.origin.y, roz = ray.origin.z;
    imp_float rdx = ray.direction.x, rdy = ray.direction.y, rdz = ray.direction.z;
    
    imp_float ABx = Bx - Ax, ABy = By - Ay, ABz = Bz - Az;
    imp_float ACx = Cx - Ax, ACy = Cy - Ay, ACz = Cz - Az;

    imp_float qx = rdy*ACz - rdz*ACy, qy = rdz*ACx - rdx*ACz, qz = rdx*ACy - rdy*ACx;

    imp_float ipf = ABx*qx + ABy*qy + ABz*qz;

    if (ipf < _eps_impact_factor) return IMP_FLOAT_INF;

    imp_float iipf = 1/ipf;

    imp_float dx = rox - Ax, dy = roy - Ay, dz = roz - Az;

    intersection_data.beta = (dx*qx + dy*qy + dz*qz)*iipf;

    if (intersection_data.beta < _eps_coordinates || intersection_data.beta > 1 - _eps_coordinates)
        return IMP_FLOAT_INF;

    imp_float rx = dy*ABz - dz*ABy, ry = dz*ABx - dx*ABz, rz = dx*ABy - dy*ABx;
    intersection_data.gamma = (rdx*rx + rdy*ry + rdz*rz)*iipf;

    if (intersection_data.gamma < _eps_coordinates || (intersection_data.alpha = 1 - (intersection_data.beta + intersection_data.gamma)) < _eps_coordinates)
        return IMP_FLOAT_INF;

    imp_float distance = (ACx*rx + ACy*ry + ACz*rz)*iipf;

    if (distance <= 0)
        return IMP_FLOAT_INF;

    return distance;
}

TriangleMesh& TriangleMesh::applyTransformation(const LinearTransformation& transformation)
{
    _vertices.rows(0, 2) = transformation.getMatrix().toArma3x3Matrix()*_vertices.rows(0, 2);

    if (_has_face_normals)
	{
        _face_normals = arma::normalise(transformation.getNormalTransformMatrix().toArma3x3Matrix()*_face_normals);
	}

    if (_has_vertex_normals)
	{
        _vertex_normals = arma::normalise(transformation.getNormalTransformMatrix().toArma3x3Matrix()*_vertex_normals);
	}

    if (_has_vertex_tangents)
	{
        _vertex_tangents.rows(0, 2) = arma::normalise(transformation.getMatrix().toArma3x3Matrix()*_vertex_tangents.rows(0, 2));
	}

    _has_aabb = false;

	return *this;
}

TriangleMesh& TriangleMesh::applyTransformation(const AffineTransformation& transformation)
{
    _vertices = transformation.getMatrix().toArma4x4Matrix()*_vertices;

    if (_has_face_normals)
	{
        _face_normals = arma::normalise(transformation.getNormalTransformMatrix().toArma3x3Matrix()*_face_normals);
	}

    if (_has_vertex_normals)
	{
        _vertex_normals = arma::normalise(transformation.getNormalTransformMatrix().toArma3x3Matrix()*_vertex_normals);
	}

    if (_has_vertex_tangents)
	{
        _vertex_tangents.rows(0, 2) = arma::normalise(transformation.getMatrix().getLinearPart().toArma3x3Matrix()*_vertex_tangents.rows(0, 2));
	}
	
    _has_aabb = false;

	return *this;
}

TriangleMesh& TriangleMesh::applyWindowingTransformation(const AffineTransformation& transformation)
{
    assert(_is_homogenized);

    _vertices.rows(0, 1) = transformation.getMatrix().toArma4x4Matrix().submat(0, 0, 1, 3)*_vertices;
	
    _has_vertex_normals = false;
    _has_vertex_tangents = false;
    _has_face_normals = false;
    _has_aabb = false;

	return *this;
}

TriangleMesh& TriangleMesh::applyTransformation(const ProjectiveTransformation& transformation)
{
    _vertices = transformation.getMatrix()*_vertices;
	
	_face_normals.clear();
	_vertex_normals.clear();

    _is_homogenized = false;
    _has_vertex_normals = false;
    _has_vertex_tangents = false;
    _has_face_normals = false;
    _has_aabb = false;

	return *this;
}

Point TriangleMesh::getVertex(imp_uint idx) const
{
    return Point(_vertices(0, idx), _vertices(1, idx), _vertices(2, idx));
}

Vector TriangleMesh::getVertexNormal(imp_uint idx) const
{
	assert(_has_vertex_normals);
    return Vector(_vertex_normals(0, idx), _vertex_normals(1, idx), _vertex_normals(2, idx));
}

void TriangleMesh::getVertexTangents(imp_uint idx, Vector& tangent, Vector& bitangent) const
{
	assert(_has_vertex_tangents);

    tangent.setComponents(_vertex_tangents(0, idx), _vertex_tangents(1, idx), _vertex_tangents(2, idx));
	bitangent = (getVertexNormal(idx).cross(tangent))*_vertex_tangents(3, idx);
}

void TriangleMesh::getVertexNormalsForFace(imp_uint face_idx, Vector vertex_normals[3]) const
{
	assert(_has_vertex_normals);

    imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

    vertex_normals[0].setComponents(_vertex_normals(0, i), _vertex_normals(1, i), _vertex_normals(2, i));
    vertex_normals[1].setComponents(_vertex_normals(0, j), _vertex_normals(1, j), _vertex_normals(2, j));
    vertex_normals[2].setComponents(_vertex_normals(0, k), _vertex_normals(1, k), _vertex_normals(2, k));
}

void TriangleMesh::getVertexTangentsForFace(imp_uint face_idx, Vector tangents[3], Vector bitangents[3]) const
{
	assert(_has_vertex_tangents);

    imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

	getVertexTangents(i, tangents[0], bitangents[0]);
	getVertexTangents(j, tangents[1], bitangents[1]);
	getVertexTangents(k, tangents[2], bitangents[2]);
}

void TriangleMesh::getVertexData3(imp_uint idx,
								  imp_float& data_0,
								  imp_float& data_1,
								  imp_float& data_2) const
{
	assert(_has_vertex_data_3);

    data_0 = _vertex_data_3(0, idx);
    data_1 = _vertex_data_3(1, idx);
    data_2 = _vertex_data_3(2, idx);
}

Triangle TriangleMesh::getFace(imp_uint face_idx) const
{
    return Triangle(getVertex(_faces(0, face_idx)), getVertex(_faces(1, face_idx)), getVertex(_faces(2, face_idx)));
}

void TriangleMesh::getFaceVertices(imp_uint face_idx, Point vertices[3]) const
{
    imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

    vertices[0].moveTo(_vertices(0, i), _vertices(1, i), _vertices(2, i));
    vertices[1].moveTo(_vertices(0, j), _vertices(1, j), _vertices(2, j));
    vertices[2].moveTo(_vertices(0, k), _vertices(1, k), _vertices(2, k));
}

Vector TriangleMesh::getFaceNormal(imp_uint face_idx) const
{
	assert(_has_face_normals);
    return Vector(_face_normals(0, face_idx), _face_normals(1, face_idx), _face_normals(2, face_idx));
}

Vector TriangleMesh::getInterpolatedVertexNormal(imp_uint face_idx, imp_float alpha, imp_float beta, imp_float gamma) const
{
    assert(_has_vertex_normals);

    imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);
	
	return Vector(alpha*_vertex_normals(0, i) + beta*_vertex_normals(0, j) + gamma*_vertex_normals(0, k),
				  alpha*_vertex_normals(1, i) + beta*_vertex_normals(1, j) + gamma*_vertex_normals(1, k),
				  alpha*_vertex_normals(2, i) + beta*_vertex_normals(2, j) + gamma*_vertex_normals(2, k)).getNormalized();
}

Vector TriangleMesh::getInterpolatedVertexNormal(const MeshIntersectionData& intersection_data) const
{
    assert(_has_vertex_normals);

    imp_uint i = _faces(0, intersection_data.face_id), j = _faces(1, intersection_data.face_id), k = _faces(2, intersection_data.face_id);
	
	return Vector(intersection_data.alpha*_vertex_normals(0, i) + intersection_data.beta*_vertex_normals(0, j) + intersection_data.gamma*_vertex_normals(0, k),
				  intersection_data.alpha*_vertex_normals(1, i) + intersection_data.beta*_vertex_normals(1, j) + intersection_data.gamma*_vertex_normals(1, k),
				  intersection_data.alpha*_vertex_normals(2, i) + intersection_data.beta*_vertex_normals(2, j) + intersection_data.gamma*_vertex_normals(2, k)).getNormalized();
}

void TriangleMesh::getInterpolatedVertexTangents(imp_uint face_idx,
												 imp_float alpha, imp_float beta, imp_float gamma,
												 Vector& tangent, Vector& bitangent) const
{
    assert(_has_vertex_tangents);

	Vector tangents[3], bitangents[3];

	getVertexTangentsForFace(face_idx, tangents, bitangents);

	tangent = (alpha*tangents[0] + beta*tangents[1] + gamma*tangents[2]).getNormalized();
	bitangent = (alpha*bitangents[0] + beta*bitangents[1] + gamma*bitangents[2]).getNormalized();
}

void TriangleMesh::getInterpolatedVertexTangents(const MeshIntersectionData& intersection_data,
												 Vector& tangent, Vector& bitangent) const
{
    assert(_has_vertex_tangents);

	Vector tangents[3], bitangents[3];

	getVertexTangentsForFace(intersection_data.face_id, tangents, bitangents);

	tangent = (intersection_data.alpha*tangents[0] + intersection_data.beta*tangents[1] + intersection_data.gamma*tangents[2]).getNormalized();
	bitangent = (intersection_data.alpha*bitangents[0] + intersection_data.beta*bitangents[1] + intersection_data.gamma*bitangents[2]).getNormalized();
}

void TriangleMesh::getVertexData3ForFace(imp_uint face_idx,
										 imp_float data_A[3],
										 imp_float data_B[3],
										 imp_float data_C[3]) const
{
	assert(_has_vertex_data_3);

    imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

    data_A[0] = _vertex_data_3(0, i);
    data_A[1] = _vertex_data_3(1, i);
    data_A[2] = _vertex_data_3(2, i);

    data_B[0] = _vertex_data_3(0, j);
    data_B[1] = _vertex_data_3(1, j);
    data_B[2] = _vertex_data_3(2, j);

    data_C[0] = _vertex_data_3(0, k);
    data_C[1] = _vertex_data_3(1, k);
    data_C[2] = _vertex_data_3(2, k);
}

void TriangleMesh::getTextureCoordinates(imp_uint face_idx, Point2 texture_coordinates[3]) const
{
    assert(_has_texture_coordinates);

    imp_uint l = _faces(3, face_idx), m = _faces(4, face_idx), n = _faces(5, face_idx);

	texture_coordinates[0] = _texture_coordinates[l];
	texture_coordinates[1] = _texture_coordinates[m];
	texture_coordinates[2] = _texture_coordinates[n];
}

Geometry2D::Point TriangleMesh::getInterpolatedTextureCoordinates(imp_uint face_idx, imp_float alpha, imp_float beta, imp_float gamma) const
{
    assert(_has_texture_coordinates);

    imp_uint l = _faces(3, face_idx), m = _faces(4, face_idx), n = _faces(5, face_idx);
	
	return Point2(_texture_coordinates[l] + (_texture_coordinates[m] - _texture_coordinates[l])*beta + (_texture_coordinates[n] - _texture_coordinates[l])*gamma);
}

Geometry2D::Point TriangleMesh::getInterpolatedTextureCoordinates(const MeshIntersectionData& intersection_data) const
{
    assert(_has_texture_coordinates);
	
    imp_uint l = _faces(3, intersection_data.face_id), m = _faces(4, intersection_data.face_id), n = _faces(5, intersection_data.face_id);
	
	return Point2(_texture_coordinates[l] + (_texture_coordinates[m] - _texture_coordinates[l])*intersection_data.beta + (_texture_coordinates[n] - _texture_coordinates[l])*intersection_data.gamma);
}

Geometry2D::Point TriangleMesh::getProjectedVertex(imp_uint idx,
												   imp_float image_width,
												   imp_float image_height,
												   imp_float inverse_image_width_at_unit_distance_from_camera,
												   imp_float inverse_image_height_at_unit_distance_from_camera) const
{
	imp_float normalization = -1/_vertices(2, idx);

    return Point2(image_width*(_vertices(0, idx)*normalization*inverse_image_width_at_unit_distance_from_camera + 0.5f),
                  image_height*(_vertices(1, idx)*normalization*inverse_image_height_at_unit_distance_from_camera + 0.5f));
}

Geometry2D::Triangle TriangleMesh::getProjectedFace(imp_uint face_idx,
													imp_float image_width,
													imp_float image_height,
													imp_float inverse_image_width_at_unit_distance_from_camera,
													imp_float inverse_image_height_at_unit_distance_from_camera) const
{
	imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

    imp_float normalization = -1/_vertices(2, i);
    Point2 vertex_A(image_width*(_vertices(0, i)*normalization*inverse_image_width_at_unit_distance_from_camera + 0.5f),
                    image_height*(_vertices(1, i)*normalization*inverse_image_height_at_unit_distance_from_camera + 0.5f));

    normalization = -1/_vertices(2, j);
    Point2 vertex_B(image_width*(_vertices(0, j)*normalization*inverse_image_width_at_unit_distance_from_camera + 0.5f),
                    image_height*(_vertices(1, j)*normalization*inverse_image_height_at_unit_distance_from_camera + 0.5f));

    normalization = -1/_vertices(2, k);
    Point2 vertex_C(image_width*(_vertices(0, k)*normalization*inverse_image_width_at_unit_distance_from_camera + 0.5f),
                    image_height*(_vertices(1, k)*normalization*inverse_image_height_at_unit_distance_from_camera + 0.5f));

    return Triangle2(vertex_A, vertex_B, vertex_C);
}

bool TriangleMesh::allZAbove(imp_float z_low) const
{
	return arma::all(_vertices.row(2) > z_low);
}

bool TriangleMesh::isInsideParallelViewVolume() const
{
	assert(_has_aabb);
	AxisAlignedBox parallel_view_volume(Point(-1, -1, -1), Point(1, 1, 0));
	return (_aabb.intersects(parallel_view_volume)) && (!_aabb.encloses(parallel_view_volume));
}

bool TriangleMesh::faceFacesOrigin(imp_uint face_idx) const
{
	assert(_has_face_normals);

	imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

	return (_vertices(0, i)*_face_normals(0, face_idx) +
		    _vertices(1, i)*_face_normals(1, face_idx) +
		    _vertices(2, i)*_face_normals(2, face_idx)) < 0;
}

bool TriangleMesh::isOutsideViewFrustum(const Plane& lower_plane,
										const Plane& upper_plane,
										const Plane& left_plane,
										const Plane& right_plane,
										imp_float far_plane_distance) const
{
	assert(_has_aabb);

	// Plane normals are assumed to point outwards from the frustum volume,
	// and the AABB is assumed to be expressed in the camera coordinate system.

	if (lower_plane.hasOnPositiveSide(_aabb.upper_corner))
		return true;

	if (upper_plane.hasOnPositiveSide(_aabb.lower_corner))
		return true;

	if (left_plane.hasOnPositiveSide(_aabb.upper_corner))
		return true;

	if (right_plane.hasOnPositiveSide(_aabb.lower_corner))
		return true;

	if (far_plane_distance < -_aabb.lower_corner.z)
		return true;

	return false;
}

std::vector<imp_uint> TriangleMesh::getPotentiallyIntersectedFaceIndices(const Ray& ray) const
{
	assert(_has_aabb);
    return _bounding_volume_hierarchy.getPotentiallyIntersectedObjectIDs(ray);
}

void TriangleMesh::addPotentiallyIntersectedFaceIndices(const Point2& pixel_center, std::list<imp_uint>& face_indices) const
{
    _bounding_area_hierarchy.addPotentiallyIntersectedObjectIDs(pixel_center, face_indices);
}

const AxisAlignedBox& TriangleMesh::getAABB() const
{
	assert(_has_aabb);
    return _aabb;
}

Point TriangleMesh::getCentroid() const
{
	const arma::Col<imp_float>& mean = arma::mean(_vertices, 1);
	return Point(mean(0), mean(1), mean(2));
}

imp_uint TriangleMesh::getNumberOfVertices() const
{
    return static_cast<imp_uint>(_vertices.n_cols);
}

imp_uint TriangleMesh::getNumberOfFaces() const
{
    return static_cast<imp_uint>(_faces.n_cols);
}

imp_uint TriangleMesh::getNumberOfTextureCoordinates() const
{
    return (_has_texture_coordinates)? static_cast<imp_uint>(_texture_coordinates.size()) : 0;
}

std::string TriangleMesh::getVerticesString() const
{
    std::ostringstream string_stream;
    string_stream << _vertices.rows(0, 2);
    return string_stream.str();
}

std::string TriangleMesh::get4SpaceVerticesString() const
{
    std::ostringstream string_stream;
    string_stream << _vertices;
    return string_stream.str();
}

std::string TriangleMesh::getFacesString() const
{
    std::ostringstream string_stream;
    string_stream << _faces;
    return string_stream.str();
}

bool TriangleMesh::isHomogenized() const
{
    return _is_homogenized;
}

bool TriangleMesh::hasFaceNormals() const
{
    return _has_face_normals;
}

bool TriangleMesh::hasVertexNormals() const
{
    return _has_vertex_normals;
}

bool TriangleMesh::hasTextureCoordinates() const
{
    return _has_texture_coordinates;
}

bool TriangleMesh::hasVertexTangents() const
{
    return _has_vertex_tangents;
}

bool TriangleMesh::hasAABB() const
{
    return _has_aabb;
}

void TriangleMesh::saveAs(const std::string& filename) const
{
    imp_uint length = static_cast<imp_uint>(filename.length());
    assert(length > 4 &&
           filename.substr(length-4, 4) == ".obj");

    std::ofstream outfile(filename);
    assert(outfile);

    std::string material_name = "default";

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

    if (_has_vertex_normals)
    {
        for (idx = 0; idx < n_vertices; idx++)
        {
            outfile << "vn " << _vertex_normals(0, idx) << " " << 
                                _vertex_normals(1, idx) << " " <<
                                _vertex_normals(2, idx) <<
                                std::endl;
        }
        
        outfile << std::endl;

        for (idx = 0; idx < n_faces; idx++)
        {
            outfile << "f " << _faces(0, idx)+1 << "//" << _faces(0, idx)+1 << " " <<
                               _faces(1, idx)+1 << "//" << _faces(1, idx)+1 << " " <<
                               _faces(2, idx)+1 << "//" << _faces(2, idx)+1 << std::endl;
        }
    }
    else
    {

        for (idx = 0; idx < n_faces; idx++)
        {
            outfile << "f " << _faces(0, idx)+1 << " " << 
                               _faces(1, idx)+1 << " " <<
                               _faces(2, idx)+1 << std::endl;
        }
    }

    outfile.close();
}

} // Geometry3D
} // Impact
