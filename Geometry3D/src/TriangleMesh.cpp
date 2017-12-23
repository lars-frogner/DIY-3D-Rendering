#include "TriangleMesh.hpp"
#include "string_util.hpp"
#include "BAHNode.hpp"
#include <cstdlib>
#include <cmath>
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
	  _has_vertex_texture_coordinate_indices(false),
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

	mesh._topology.initializeFaces(n_faces, mesh._has_texture_coordinates);

	if (mesh._has_texture_coordinates)
	{
		for (idx = 0; idx < n_faces; idx++)
		{
			splitted = string_util::split(faces[idx]);
			n_values = static_cast<imp_uint>(splitted.size());

			if (n_values == 3)
			{
				mesh._topology.setNextFace(_getFaceIndexFromObjString(splitted[0], n_vertices),
										   _getFaceIndexFromObjString(splitted[1], n_vertices),
										   _getFaceIndexFromObjString(splitted[2], n_vertices),
										   _getFaceTextureIndexFromObjString(splitted[0], n_texture_coords),
										   _getFaceTextureIndexFromObjString(splitted[1], n_texture_coords),
										   _getFaceTextureIndexFromObjString(splitted[2], n_texture_coords));
			}
			else if (n_values == 4)
			{
				const imp_uint& i = _getFaceIndexFromObjString(splitted[0], n_vertices);
				const imp_uint& k = _getFaceIndexFromObjString(splitted[2], n_vertices);
				const imp_uint& l = _getFaceTextureIndexFromObjString(splitted[0], n_texture_coords);
				const imp_uint& n = _getFaceTextureIndexFromObjString(splitted[2], n_texture_coords);

				mesh._topology.setNextFace(i,
										   _getFaceIndexFromObjString(splitted[1], n_vertices),
										   k,
										   l,
										   _getFaceTextureIndexFromObjString(splitted[1], n_texture_coords),
										   n);

				mesh._topology.setNextFace(k,
										   _getFaceIndexFromObjString(splitted[3], n_vertices),
										   i,
										   n,
										   _getFaceTextureIndexFromObjString(splitted[3], n_texture_coords),
										   l);
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
		for (idx = 0; idx < n_faces; idx++)
		{
			splitted = string_util::split(faces[idx]);
			n_values = static_cast<imp_uint>(splitted.size());

			if (n_values == 3)
			{
				mesh._topology.setNextFace(_getFaceIndexFromObjString(splitted[0], n_vertices),
										   _getFaceIndexFromObjString(splitted[1], n_vertices),
										   _getFaceIndexFromObjString(splitted[2], n_vertices));
			}
			else if (n_values == 4)
			{
				const imp_uint& i = _getFaceIndexFromObjString(splitted[0], n_vertices);
				const imp_uint& k = _getFaceIndexFromObjString(splitted[2], n_vertices);

				mesh._topology.setNextFace(i,
										   _getFaceIndexFromObjString(splitted[1], n_vertices),
									       k);

				mesh._topology.setNextFace(k,
										   _getFaceIndexFromObjString(splitted[3], n_vertices),
										   i);
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

    triangle_mesh._topology.addFace(0, 1, 2);

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

	box_mesh._topology.addFace( 0,  9,  3);
	box_mesh._topology.addFace( 3,  9,  6);
	box_mesh._topology.addFace(12, 15, 21);
	box_mesh._topology.addFace(21, 15, 18);
	box_mesh._topology.addFace( 1,  4, 13);
	box_mesh._topology.addFace(13,  4, 16);
	box_mesh._topology.addFace(19,  7, 22);
	box_mesh._topology.addFace(22,  7, 10);
	box_mesh._topology.addFace( 2, 14, 11);
	box_mesh._topology.addFace(11, 14, 23);
	box_mesh._topology.addFace( 5,  8, 17);
	box_mesh._topology.addFace(17,  8, 20);
	
	box_mesh.computeFaceNormals();
	box_mesh.computeVertexNormals();

    return box_mesh;
}

TriangleMesh TriangleMesh::manifoldBox(const Box& box_obj)
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

	const Point& center_1 = corners[0] + (corners[2] - corners[0])*0.5f;
	const Point& center_2 = corners[4] + (corners[6] - corners[4])*0.5f;
	const Point& center_3 = corners[4] + (corners[3] - corners[4])*0.5f;
	const Point& center_4 = corners[1] + (corners[6] - corners[1])*0.5f;
	const Point& center_5 = corners[0] + (corners[5] - corners[0])*0.5f;
	const Point& center_6 = corners[3] + (corners[6] - corners[3])*0.5f;

    imp_float x8 = center_1.x, y8 = center_1.y, z8 = center_1.z;
    imp_float x9 = center_2.x, y9 = center_2.y, z9 = center_2.z;
    imp_float x10 = center_3.x, y10 = center_3.y, z10 = center_3.z;
    imp_float x11 = center_4.x, y11 = center_4.y, z11 = center_4.z;
    imp_float x12 = center_5.x, y12 = center_5.y, z12 = center_5.z;
    imp_float x13 = center_6.x, y13 = center_6.y, z13 = center_6.z;

    box_mesh._vertices = {{x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13},
                          {y0, y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12, y13},
                          {z0, z1, z2, z3, z4, z5, z6, z7, z8, z9, z10, z11, z12, z13},
                          { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,   1,   1,   1,   1}};

	box_mesh._topology.addFace(8, 1, 0);
	box_mesh._topology.addFace(8, 2, 1);
	box_mesh._topology.addFace(8, 3, 2);
	box_mesh._topology.addFace(8, 0, 3);

	box_mesh._topology.addFace(9, 5, 6);
	box_mesh._topology.addFace(9, 6, 7);
	box_mesh._topology.addFace(9, 7, 4);
	box_mesh._topology.addFace(9, 4, 5);

	box_mesh._topology.addFace(10, 0, 4);
	box_mesh._topology.addFace(10, 3, 0);
	box_mesh._topology.addFace(10, 7, 3);
	box_mesh._topology.addFace(10, 4, 7);

	box_mesh._topology.addFace(11, 1, 2);
	box_mesh._topology.addFace(11, 2, 6);
	box_mesh._topology.addFace(11, 6, 5);
	box_mesh._topology.addFace(11, 5, 1);

	box_mesh._topology.addFace(12, 5, 4);
	box_mesh._topology.addFace(12, 1, 5);
	box_mesh._topology.addFace(12, 0, 1);
	box_mesh._topology.addFace(12, 4, 0);

	box_mesh._topology.addFace(13, 6, 2);
	box_mesh._topology.addFace(13, 2, 3);
	box_mesh._topology.addFace(13, 3, 7);
	box_mesh._topology.addFace(13, 7, 6);
	
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

	room_mesh._topology.addFace( 9,  3,  9);
	room_mesh._topology.addFace( 3,  6,  9);
	room_mesh._topology.addFace(12, 21, 15);
	room_mesh._topology.addFace(21, 18, 15);
	room_mesh._topology.addFace( 1, 13,  4);
	room_mesh._topology.addFace(13, 16,  4);
	room_mesh._topology.addFace(19, 22,  7);
	room_mesh._topology.addFace(22, 10,  7);
	room_mesh._topology.addFace( 2, 11, 14);
	room_mesh._topology.addFace(11, 23, 14);
	room_mesh._topology.addFace( 5, 17,  8);
	room_mesh._topology.addFace(17, 20,  8);
	
	room_mesh.computeFaceNormals();
	room_mesh.computeVertexNormals();

    return room_mesh;
}

TriangleMesh TriangleMesh::sheet(const Point& center, const Vector& normal, const Vector& width_vector, imp_float height)
{
    TriangleMesh sheet_mesh;

	const Vector& new_normal = normal.getNormalized();
	const Vector& height_vector = new_normal.getUnitNormalWith(width_vector)*height;
	const Point& origin = center - width_vector*0.5f - height_vector*0.5f;

    const Point& corner_1 = origin;
    const Point& corner_2 = corner_1 + width_vector;
    const Point& corner_3 = corner_2 + height_vector;
    const Point& corner_4 = corner_1 + height_vector;

    sheet_mesh._vertices = {{corner_1.x, corner_2.x, corner_3.x, corner_4.x},
                            {corner_1.y, corner_2.y, corner_3.y, corner_4.y},
                            {corner_1.z, corner_2.z, corner_3.z, corner_4.z},
                            {         1,          1,          1,          1}};
	
	sheet_mesh._topology.addFace(1, 2, 0);
	sheet_mesh._topology.addFace(0, 2, 3);

	sheet_mesh._face_normals = {{new_normal.x, new_normal.x},
								{new_normal.y, new_normal.y},
								{new_normal.z, new_normal.z}};

	sheet_mesh._vertex_normals = {{new_normal.x, new_normal.x, new_normal.x, new_normal.x},
								  {new_normal.y, new_normal.y, new_normal.y, new_normal.y},
								  {new_normal.z, new_normal.z, new_normal.z, new_normal.z}};
	
	sheet_mesh._has_face_normals = true;
	sheet_mesh._has_vertex_normals = true;

	sheet_mesh._texture_coordinates = {Point2(0, 0), Point2(1, 0), Point2(1, 1), Point2(0, 1)};

    sheet_mesh._topology.duplicateFaceIndicesForTextureCoordinates();
	sheet_mesh._has_texture_coordinates = true;
	sheet_mesh.computeVertexTextureCoordinateIndices();

	sheet_mesh.computeTangentVectors();

    return sheet_mesh;
}

TriangleMesh TriangleMesh::symmetricSheet(const Point& center, const Vector& normal, const Vector& width_vector, imp_float height)
{
    TriangleMesh sheet_mesh;

	const Vector& new_normal = normal.getNormalized();
	const Vector& height_vector = new_normal.getUnitNormalWith(width_vector)*height;
	const Point& origin = center - width_vector*0.5f - height_vector*0.5f;

    const Point& corner_1 = origin;
    const Point& corner_2 = corner_1 + width_vector;
    const Point& corner_3 = corner_2 + height_vector;
    const Point& corner_4 = corner_1 + height_vector;
	const Point& midpoint = origin + width_vector*0.5f + height_vector*0.5f;

    sheet_mesh._vertices = {{corner_1.x, corner_2.x, corner_3.x, corner_4.x, midpoint.x},
                            {corner_1.y, corner_2.y, corner_3.y, corner_4.y, midpoint.y},
                            {corner_1.z, corner_2.z, corner_3.z, corner_4.z, midpoint.z},
                            {         1,          1,          1,          1,		  1}};
	
	sheet_mesh._topology.addFace(4, 0, 1);
	sheet_mesh._topology.addFace(4, 1, 2);
	sheet_mesh._topology.addFace(4, 2, 3);
	sheet_mesh._topology.addFace(4, 3, 0);

	sheet_mesh._face_normals = {{new_normal.x, new_normal.x, new_normal.x, new_normal.x},
								{new_normal.y, new_normal.y, new_normal.y, new_normal.y},
								{new_normal.z, new_normal.z, new_normal.z, new_normal.z}};

	sheet_mesh._vertex_normals = {{new_normal.x, new_normal.x, new_normal.x, new_normal.x, new_normal.x},
								  {new_normal.y, new_normal.y, new_normal.y, new_normal.y, new_normal.y},
								  {new_normal.z, new_normal.z, new_normal.z, new_normal.z, new_normal.z}};
	
	sheet_mesh._has_face_normals = true;
	sheet_mesh._has_vertex_normals = true;

	sheet_mesh._texture_coordinates = {Point2(0, 0), Point2(1, 0), Point2(1, 1), Point2(0, 1), Point2(0.5f, 0.5f)};

    sheet_mesh._topology.duplicateFaceIndicesForTextureCoordinates();
	sheet_mesh._has_texture_coordinates = true;
	sheet_mesh.computeVertexTextureCoordinateIndices();

	sheet_mesh.computeTangentVectors();

    return sheet_mesh;
}

TriangleMesh TriangleMesh::twoSidedSheet(const Point& center, const Vector& normal, const Vector& width_vector, imp_float height)
{
    TriangleMesh sheet_mesh;

	const Vector& new_normal = normal.getNormalized();
	const Vector& height_vector = new_normal.getUnitNormalWith(width_vector)*height;
	const Point& origin = center - width_vector*0.5f - height_vector*0.5f;

    const Point& corner_1 = origin;
    const Point& corner_2 = corner_1 + width_vector;
    const Point& corner_3 = corner_2 + height_vector;
    const Point& corner_4 = corner_1 + height_vector;

    sheet_mesh._vertices = {{corner_1.x, corner_2.x, corner_3.x, corner_4.x},
                            {corner_1.y, corner_2.y, corner_3.y, corner_4.y},
                            {corner_1.z, corner_2.z, corner_3.z, corner_4.z},
                            {         1,          1,          1,          1}};
	
	sheet_mesh._topology.addFace(1, 2, 0);
	sheet_mesh._topology.addFace(0, 2, 3);
	sheet_mesh._topology.addFace(1, 0, 2);
	sheet_mesh._topology.addFace(2, 0, 3);

	sheet_mesh._face_normals = {{new_normal.x, new_normal.x, new_normal.x, new_normal.x},
								{new_normal.y, new_normal.y, new_normal.y, new_normal.y},
								{new_normal.z, new_normal.z, new_normal.z, new_normal.z}};

	sheet_mesh._vertex_normals = {{new_normal.x, new_normal.x, new_normal.x, new_normal.x},
								  {new_normal.y, new_normal.y, new_normal.y, new_normal.y},
								  {new_normal.z, new_normal.z, new_normal.z, new_normal.z}};
	
	sheet_mesh._has_face_normals = true;
	sheet_mesh._has_vertex_normals = true;

    return sheet_mesh;
}

TriangleMesh TriangleMesh::sphere(const Sphere& sphere_obj, imp_uint resolution, imp_uint texture_mapping_mode /* = 0 */)
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
    sphere_mesh._vertex_normals(1, 0) = 1;
    sphere_mesh._vertex_normals(2, 0) = 0;
    sphere_mesh._vertices(0, 0) = x0;
    sphere_mesh._vertices(1, 0) = y0 + r;
    sphere_mesh._vertices(2, 0) = z0;

    n = 1;
    for (i = 1; i < n_lat-1; i++) {
        for (j = 0; j < n_lon; j++)
        {
            theta = i*dtheta;
            phi = j*dphi;
            sin_theta = sin(theta);
			
            sphere_mesh._vertex_normals(0, n) = sin_theta*sin(phi);
            sphere_mesh._vertex_normals(1, n) = cos(theta);
            sphere_mesh._vertex_normals(2, n) = sin_theta*cos(phi);
			
            sphere_mesh._vertices(0, n) = x0 + r*sphere_mesh._vertex_normals(0, n);
            sphere_mesh._vertices(1, n) = y0 + r*sphere_mesh._vertex_normals(1, n);
            sphere_mesh._vertices(2, n) = z0 + r*sphere_mesh._vertex_normals(2, n);

            n++;
        }
    }
    
    // Bottom vertex
    sphere_mesh._vertex_normals(0, n) = 0;
    sphere_mesh._vertex_normals(1, n) = -1;
    sphere_mesh._vertex_normals(2, n) = 0;
    sphere_mesh._vertices(0, n) = x0;
    sphere_mesh._vertices(1, n) = y0 - r;
    sphere_mesh._vertices(2, n) = z0;

	sphere_mesh._has_vertex_normals = true;

	// Texture coordinates

	assert(texture_mapping_mode == 0 || texture_mapping_mode == 1 || texture_mapping_mode == 2);

	if (texture_mapping_mode == 1) // Cylindrical mapping
	{
		sphere_mesh._texture_coordinates.emplace_back(0.0f, 1.0f);

		for (i = 1; i < n_lat-1; i++) {
			for (j = 0; j < n_lon; j++)
			{
				theta = i*dtheta;
				phi = j*dphi;

				sphere_mesh._texture_coordinates.emplace_back(phi/IMP_TWO_PI, (1.0f + cos(theta))*0.5f);
			}
		}

		sphere_mesh._texture_coordinates.emplace_back(0.0f, 0.0f);

		// Add additional texture coordinates for u = 1 to ensure correct interpolation from u <~ 1 to u = 0
		sphere_mesh._texture_coordinates.emplace_back(1.0f, 1.0f);

		for (i = 1; i < n_lat-1; i++)
		{
			theta = i*dtheta;
			sphere_mesh._texture_coordinates.emplace_back(1.0f, (1.0f + cos(theta))*0.5f);
		}

		sphere_mesh._texture_coordinates.emplace_back(1.0f, 0.0f);

		sphere_mesh._has_texture_coordinates = true;
	}
	else if (texture_mapping_mode == 2) // Equirectangular mapping
	{
		sphere_mesh._texture_coordinates.emplace_back(0.0f, 1.0f);

		for (i = 1; i < n_lat-1; i++) {
			for (j = 0; j < n_lon; j++)
			{
				theta = i*dtheta;
				phi = j*dphi;
				
				sphere_mesh._texture_coordinates.emplace_back(phi/IMP_TWO_PI, 1.0f - theta/IMP_PI);
			}
		}

		sphere_mesh._texture_coordinates.emplace_back(0.0f, 0.0f);

		// Add additional texture coordinates for u = 1 to ensure correct interpolation from u <~ 1 to u = 0
		sphere_mesh._texture_coordinates.emplace_back(1.0f, 1.0f);

		for (i = 1; i < n_lat-1; i++)
		{
			theta = i*dtheta;
			sphere_mesh._texture_coordinates.emplace_back(1.0f, 1.0f - theta/IMP_PI);
		}

		sphere_mesh._texture_coordinates.emplace_back(1.0f, 0.0f);
		
		for (j = 0; j < n_lon; j++)
		{
			phi = j*dphi;
				
			sphere_mesh._texture_coordinates.emplace_back(phi/IMP_TWO_PI, 1.0f);
		}
		
		for (j = 0; j < n_lon; j++)
		{
			phi = j*dphi;
				
			sphere_mesh._texture_coordinates.emplace_back(phi/IMP_TWO_PI, 0.0f);
		}

		sphere_mesh._has_texture_coordinates = true;
	}

	// Faces

	sphere_mesh._topology.initializeFaces(n_faces, true);

	m = n+1;
    
    // Top cone
    for (j = 1; j < n_lon; j++)
    {
        sphere_mesh._topology.setNextFace(j, j + 1, 0,
										  j, j + 1, m + n_lat + j - 1);
    }
    sphere_mesh._topology.setNextFace(n_lon, 1,  0,
									  n_lon, m + 1,  m + n_lat + n_lon - 1);
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

            sphere_mesh._topology.setNextFace(current, right, above_right,
											  current, right, above_right);

            sphere_mesh._topology.setNextFace(above_right, above, current,
											  above_right, above, current);
        }
        
        j = n_lon - 1;

        current =     offset + j;
        right =       offset;
        above =       offset_prev + j;
        above_right = offset_prev;

        sphere_mesh._topology.setNextFace(current, right, above_right,
										  current, m + 1, m);

        sphere_mesh._topology.setNextFace(above_right, above, current,
										  m, above, current);
		m++;
    }

    // Bottom cone
    offset = 1 + (n_lat - 3)*n_lon;
    for (j = 0; j < n_lon-1; j++)
    {
        sphere_mesh._topology.setNextFace(offset + j + 1, offset + j, n_vertices - 1,
										  offset + j + 1, offset + j, m + 2 + n_lon + j);
    }
    sphere_mesh._topology.setNextFace(offset, offset + n_lon - 1, n_vertices - 1,
									  m, offset + n_lon - 1, m + 2 + n_lon + n_lon - 1);

	if (!sphere_mesh._has_texture_coordinates)
		sphere_mesh._topology.removeTextureCoordinateIndices();

	sphere_mesh.computeFaceNormals();

	if (sphere_mesh._has_texture_coordinates)
	{
		sphere_mesh.computeVertexTextureCoordinateIndices();
		sphere_mesh.computeTangentVectors();
	}

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

	sphere_mesh._has_vertex_normals = true;

	sphere_mesh._topology.initializeFaces(n_faces, false);
    
    // Top cone
    n = 0;
    for (j = 1; j < n_lon; j++)
    {
        sphere_mesh._topology.setNextFace(j, j + 1, 0);
    }
    sphere_mesh._topology.setNextFace(n_lon, 1,  0);
    
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

            sphere_mesh._topology.setNextFace(current, right, above_right);

            sphere_mesh._topology.setNextFace(above_right, above, current);
        }
        
        j = n_lon - 1;

        current =     offset + j;
        right =       offset;
        above =       offset_prev + j;
        above_right = offset_prev;

        sphere_mesh._topology.setNextFace(current, right, above_right);

        sphere_mesh._topology.setNextFace(above_right, above, current);
    }

    // Bottom cone
    offset = 1 + (n_lat - 3)*n_lon;
    for (j = 0; j < n_lon-1; j++)
    {
        sphere_mesh._topology.setNextFace(offset + j + 1, offset + j, n_vertices - 1);
    }
    sphere_mesh._topology.setNextFace(offset, offset + n_lon - 1, n_vertices - 1);

	sphere_mesh._topology.generateOppositeFaces();

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
	_has_vertex_texture_coordinate_indices = false;
	_topology.invalidateAdjacencyData();
}

void TriangleMesh::addVertex(const Point& vertex)
{
    addVertex(vertex.x, vertex.y, vertex.z);
}

void TriangleMesh::addFace(imp_uint i, imp_uint j, imp_uint k)
{
	assert(!_has_texture_coordinates);

    _topology.addFace(i, j, k);
	
    _has_face_normals = false;
	_topology.invalidateAdjacencyData();
}

void TriangleMesh::addFace(imp_uint i, imp_uint j, imp_uint k,
						   imp_uint l, imp_uint m, imp_uint n)
{
	assert(_has_texture_coordinates);
	
    _topology.addFace(i, j, k, l, m, n);

    _has_face_normals = false;
	_topology.invalidateAdjacencyData();
}

void TriangleMesh::removeVertex(imp_uint idx)
{
	if (_has_face_normals)
		_topology.removeFacesContainingVertex(idx, _face_normals);
	else
		_topology.removeFacesContainingVertex(idx);

    _vertices.shed_col(idx);

	if (_has_vertex_normals)
		_vertex_normals.shed_col(idx);

	if (_has_vertex_tangents)
		_vertex_tangents.shed_col(idx);

	if (_has_vertex_data_3)
		_vertex_data_3.shed_col(idx);
	
	_has_vertex_texture_coordinate_indices = false;
    _has_aabb = false;
	_topology.invalidateAdjacencyData();
}

void TriangleMesh::removeFace(imp_uint idx)
{
    _topology.removeFace(idx);
	
	if (_has_face_normals)
		_face_normals.shed_col(idx);
	
	_topology.invalidateAdjacencyData();
}

void TriangleMesh::performLoopSubdivisions(imp_uint n_subdivisions,
										   imp_uint boundary_interpolation_mode /* = 0 */)
{
	assert(_is_homogenized);
	assert(!_has_vertex_data_3);

	imp_uint n_vertices;
	imp_uint vertex_idx;
	imp_uint n_texture_coordinates;

	arma::Mat<imp_float> new_vertices;
	imp_uint approx_n_new_vertices;
	imp_uint added_vertex_idx;
	imp_uint added_texture_coordinate_idx;
	imp_uint edge_vertex_1, edge_vertex_2, edge_vertex_3;
	imp_uint edge_texture_coordinate_idx_1, edge_texture_coordinate_idx_2, edge_texture_coordinate_idx_3;
	std::map<std::pair<imp_uint, imp_uint>, imp_uint> added_edge_vertices, added_texture_coordinate_indices;

	std::list<imp_uint> neighbour_vertices;
	std::list<imp_uint>::const_iterator neighbour_vertex;
	imp_uint n_neighbour_vertices;
	imp_uint n_adjacent_faces;
	imp_float neighbour_weight, self_weight;
	
	imp_uint n_faces;
	imp_uint face_idx;
	imp_uint adjacent_faces[3];
	imp_uint opposite_vertices[3];
	imp_uint i, j, k;
	imp_uint l, m, n;
	imp_uint l_adjacent, m_adjacent, n_adjacent;

	assert(boundary_interpolation_mode == 0 || boundary_interpolation_mode == 1);
	
	const imp_float boundary_neighbour_weight = (boundary_interpolation_mode)? 0.0f : 1.0f/8.0f;
	const imp_float boundary_self_weight	  = (boundary_interpolation_mode)? 1.0f : 6.0f/8.0f;

	for (imp_uint subdivision_i = 0; subdivision_i < n_subdivisions; subdivision_i++)
	{
		n_vertices = getNumberOfVertices();
		n_faces = getNumberOfFaces();
		n_texture_coordinates = getNumberOfTextureCoordinates();

		// Set upper bound on the number of new vertices to create.
		// A vertex is created for each edge, and there are (3 - avg_neighbours/2)*n_faces edges,
		// where avg_neighbours (between 0 and 3) is the average number of faces adjacent to a
		// face in the mesh. Setting avg_neighbours = 1 and adding a safety padding of 3 should
		// be a safe limit.
		approx_n_new_vertices = 5*n_faces/2 + 3;

		// Temporary storage of modified and new vertices
		new_vertices.set_size(3, n_vertices + approx_n_new_vertices);

		// Increase capacity of texture coordinate list
		if (_has_texture_coordinates)
			_texture_coordinates.reserve(n_texture_coordinates + approx_n_new_vertices);

		// Counter for keeping track of next available vertex index
		added_vertex_idx = n_vertices;
		
		// Counter for keeping track of next available texture coordinate index
		added_texture_coordinate_idx = n_texture_coordinates;

		// Make sure the mesh has adjacency data
		_topology.generateAdjacencyData(n_vertices);

		// Create modified versions of existing vertices
		for (vertex_idx = 0; vertex_idx < n_vertices; vertex_idx++)
		{
			// Get a list of vertices that are connected to the current vertex by a single edge
			n_adjacent_faces = _topology.findConnectedVertices(vertex_idx, neighbour_vertices);

			n_neighbour_vertices = static_cast<imp_uint>(neighbour_vertices.size());

			if (n_neighbour_vertices == n_adjacent_faces)
			{
				// The vertex is in the interior of the surface
				
				// Compute new vertex coordinates from a weighted sum of neighbour coordinates
				neighbour_weight = (n_neighbour_vertices > 3)? 0.375f/n_neighbour_vertices : 0.1875f; // 3/(8*n_neighbour_vertices) : 3/16
				self_weight = 1 - n_neighbour_vertices*neighbour_weight;

				new_vertices(0, vertex_idx) = self_weight*_vertices(0, vertex_idx);
				new_vertices(1, vertex_idx) = self_weight*_vertices(1, vertex_idx);
				new_vertices(2, vertex_idx) = self_weight*_vertices(2, vertex_idx);

				for (neighbour_vertex = neighbour_vertices.begin(); neighbour_vertex != neighbour_vertices.end(); neighbour_vertex++)
				{
					new_vertices(0, vertex_idx) += neighbour_weight*_vertices(0, *neighbour_vertex);
					new_vertices(1, vertex_idx) += neighbour_weight*_vertices(1, *neighbour_vertex);
					new_vertices(2, vertex_idx) += neighbour_weight*_vertices(2, *neighbour_vertex);
				}
			}
			else
			{
				// The vertex is on the boundary of the surface
	
				new_vertices(0, vertex_idx) = boundary_self_weight*_vertices(0, vertex_idx) + boundary_neighbour_weight*(_vertices(0, neighbour_vertices.front()) + _vertices(0, neighbour_vertices.back()));
				new_vertices(1, vertex_idx) = boundary_self_weight*_vertices(1, vertex_idx) + boundary_neighbour_weight*(_vertices(1, neighbour_vertices.front()) + _vertices(1, neighbour_vertices.back()));
				new_vertices(2, vertex_idx) = boundary_self_weight*_vertices(2, vertex_idx) + boundary_neighbour_weight*(_vertices(2, neighbour_vertices.front()) + _vertices(2, neighbour_vertices.back()));
			}
		
			// Empty list of neighbour vertices
			neighbour_vertices.clear();
		}

		// Make room for the required number of new faces
		_topology.reserveAdditionalFaces(3*n_faces);

		// Loop through all faces
		for (face_idx = 0; face_idx < n_faces; face_idx++)
		{
			// Get indices of face vertices
			_topology.getFace(face_idx, i, j, k);

			// Get texture coordinate indices if they exist
			if (_has_texture_coordinates)
				_topology.getTextureCoordinateIndices(face_idx, l, m, n);

			// Find faces adjacent to the current face, along with their
			// vertex that is not common with the vertices for the current face
			_topology.findAdjacentFaces(face_idx, adjacent_faces, opposite_vertices);

			// For each of the three face edges, add a new vertex if it
			// hasn't already been created in an earlier loop iteration

			// Edge i <-> j
			if (adjacent_faces[0] >= face_idx)
			{
				// New vertex has not been created for this edge
			
				edge_vertex_1 = added_vertex_idx;

				if (adjacent_faces[0] > face_idx)
				{
					self_weight = 0.375f; // 3/8
					neighbour_weight = 0.125f; // 1/8

					new_vertices(0, edge_vertex_1) = self_weight*(_vertices(0, i) + _vertices(0, j)) + neighbour_weight*(_vertices(0, k) + _vertices(0, opposite_vertices[0]));
					new_vertices(1, edge_vertex_1) = self_weight*(_vertices(1, i) + _vertices(1, j)) + neighbour_weight*(_vertices(1, k) + _vertices(1, opposite_vertices[0]));
					new_vertices(2, edge_vertex_1) = self_weight*(_vertices(2, i) + _vertices(2, j)) + neighbour_weight*(_vertices(2, k) + _vertices(2, opposite_vertices[0]));
				}
				else // Boundary edge
				{
					self_weight = 0.5f;

					new_vertices(0, edge_vertex_1) = self_weight*(_vertices(0, i) + _vertices(0, j));
					new_vertices(1, edge_vertex_1) = self_weight*(_vertices(1, i) + _vertices(1, j));
					new_vertices(2, edge_vertex_1) = self_weight*(_vertices(2, i) + _vertices(2, j));
				}

				 // Store index of the new vertex
				added_edge_vertices[std::make_pair(i, j)] = edge_vertex_1;

				added_vertex_idx++;

				if (_has_texture_coordinates)
				{
					edge_texture_coordinate_idx_1 = added_texture_coordinate_idx;
					
					// Create new texture coordinate
					_texture_coordinates.push_back(_texture_coordinates[l] + (_texture_coordinates[m] - _texture_coordinates[l])*0.5f);

					added_texture_coordinate_idx++;
					
					// Find corresponding texture coordinate indices for the adjacent face
					l_adjacent = _topology.getTextureCoordinateIndexForVertexIndex(adjacent_faces[0], i);
					m_adjacent = _topology.getTextureCoordinateIndexForVertexIndex(adjacent_faces[0], j);

					// If both texture coordinates differ, the edge is part of a seam
					if (l != l_adjacent && m != m_adjacent)
					{
						// Create texture coordinate for the opposite side of the seam
						_texture_coordinates.push_back(_texture_coordinates[l_adjacent] + (_texture_coordinates[m_adjacent] - _texture_coordinates[l_adjacent])*0.5f);

						// Store index of the new texture coordinate for the adjacent face
						added_texture_coordinate_indices[std::make_pair(i, j)] = added_texture_coordinate_idx;
						
						added_texture_coordinate_idx++;
					}
					else
					{
						// Store index of the new (common) texture coordinate
						added_texture_coordinate_indices[std::make_pair(i, j)] = edge_texture_coordinate_idx_1;
					}
				}
			}
			else
			{
				// New vertex has already been created for this edge

				// Get index of the new edge vertex
				edge_vertex_1 = added_edge_vertices.find(std::make_pair(j, i))->second;

				if (_has_texture_coordinates)
				{
					// Get index of the new texture coordinate
					edge_texture_coordinate_idx_1 = added_texture_coordinate_indices.find(std::make_pair(j, i))->second;
				}
			}

			// Edge j <-> k
			if (adjacent_faces[1] >= face_idx)
			{
				// New vertex has not been created for this edge
			
				edge_vertex_2 = added_vertex_idx;

				if (adjacent_faces[1] > face_idx)
				{
					self_weight = 0.375f; // 3/8
					neighbour_weight = 0.125f; // 1/8
					
					new_vertices(0, edge_vertex_2) = self_weight*(_vertices(0, j) + _vertices(0, k)) + neighbour_weight*(_vertices(0, i) + _vertices(0, opposite_vertices[1]));
					new_vertices(1, edge_vertex_2) = self_weight*(_vertices(1, j) + _vertices(1, k)) + neighbour_weight*(_vertices(1, i) + _vertices(1, opposite_vertices[1]));
					new_vertices(2, edge_vertex_2) = self_weight*(_vertices(2, j) + _vertices(2, k)) + neighbour_weight*(_vertices(2, i) + _vertices(2, opposite_vertices[1]));
				}
				else // Boundary edge
				{
					self_weight = 0.5f;

					new_vertices(0, edge_vertex_2) = self_weight*(_vertices(0, j) + _vertices(0, k));
					new_vertices(1, edge_vertex_2) = self_weight*(_vertices(1, j) + _vertices(1, k));
					new_vertices(2, edge_vertex_2) = self_weight*(_vertices(2, j) + _vertices(2, k));
				}

				 // Store index of the new vertex
				added_edge_vertices[std::make_pair(j, k)] = edge_vertex_2;

				added_vertex_idx++;

				if (_has_texture_coordinates)
				{
					edge_texture_coordinate_idx_2 = added_texture_coordinate_idx;
					
					// Create new texture coordinate
					_texture_coordinates.push_back(_texture_coordinates[m] + (_texture_coordinates[n] - _texture_coordinates[m])*0.5f);

					added_texture_coordinate_idx++;
					
					// Find corresponding texture coordinate indices for the adjacent face
					m_adjacent = _topology.getTextureCoordinateIndexForVertexIndex(adjacent_faces[1], j);
					n_adjacent = _topology.getTextureCoordinateIndexForVertexIndex(adjacent_faces[1], k);

					// If both texture coordinates differ, the edge is part of a seam
					if (m != m_adjacent && n != n_adjacent)
					{
						// Create texture coordinate for the opposite side of the seam
						_texture_coordinates.push_back(_texture_coordinates[m_adjacent] + (_texture_coordinates[n_adjacent] - _texture_coordinates[m_adjacent])*0.5f);

						// Store index of the new texture coordinate for the adjacent face
						added_texture_coordinate_indices[std::make_pair(j, k)] = added_texture_coordinate_idx;
						
						added_texture_coordinate_idx++;
					}
					else
					{
						// Store index of the new (common) texture coordinate
						added_texture_coordinate_indices[std::make_pair(j, k)] = edge_texture_coordinate_idx_2;
					}
				}
			}
			else
			{
				// New vertex has already been created for this edge
				
				// Get index of the new edge vertex
				edge_vertex_2 = added_edge_vertices.find(std::make_pair(k, j))->second;

				if (_has_texture_coordinates)
				{
					// Get index of the new texture coordinate
					edge_texture_coordinate_idx_2 = added_texture_coordinate_indices.find(std::make_pair(k, j))->second;
				}
			}

			// Edge k <-> i
			if (adjacent_faces[2] >= face_idx)
			{
				// New vertex has not been created for this edge
			
				edge_vertex_3 = added_vertex_idx;

				if (adjacent_faces[2] > face_idx)
				{
					self_weight = 0.375f; // 3/8
					neighbour_weight = 0.125f; // 1/8

					new_vertices(0, edge_vertex_3) = self_weight*(_vertices(0, k) + _vertices(0, i)) + neighbour_weight*(_vertices(0, j) + _vertices(0, opposite_vertices[2]));
					new_vertices(1, edge_vertex_3) = self_weight*(_vertices(1, k) + _vertices(1, i)) + neighbour_weight*(_vertices(1, j) + _vertices(1, opposite_vertices[2]));
					new_vertices(2, edge_vertex_3) = self_weight*(_vertices(2, k) + _vertices(2, i)) + neighbour_weight*(_vertices(2, j) + _vertices(2, opposite_vertices[2]));
				}
				else
				{
					self_weight = 0.5f;

					new_vertices(0, edge_vertex_3) = self_weight*(_vertices(0, k) + _vertices(0, i));
					new_vertices(1, edge_vertex_3) = self_weight*(_vertices(1, k) + _vertices(1, i));
					new_vertices(2, edge_vertex_3) = self_weight*(_vertices(2, k) + _vertices(2, i));
				}

				 // Store index of the new vertex
				added_edge_vertices[std::make_pair(k, i)] = edge_vertex_3;

				added_vertex_idx++;

				if (_has_texture_coordinates)
				{
					edge_texture_coordinate_idx_3 = added_texture_coordinate_idx;
					
					// Create new texture coordinate
					_texture_coordinates.push_back(_texture_coordinates[n] + (_texture_coordinates[l] - _texture_coordinates[n])*0.5f);

					added_texture_coordinate_idx++;
					
					// Find corresponding texture coordinate indices for the adjacent face
					n_adjacent = _topology.getTextureCoordinateIndexForVertexIndex(adjacent_faces[2], k);
					l_adjacent = _topology.getTextureCoordinateIndexForVertexIndex(adjacent_faces[2], i);

					// If both texture coordinates differ, the edge is part of a seam
					if (n != n_adjacent && l != l_adjacent)
					{
						// Create texture coordinate for the opposite side of the seam
						_texture_coordinates.push_back(_texture_coordinates[n_adjacent] + (_texture_coordinates[l_adjacent] - _texture_coordinates[n_adjacent])*0.5f);

						// Store index of the new texture coordinate for the adjacent face
						added_texture_coordinate_indices[std::make_pair(k, i)] = added_texture_coordinate_idx;
						
						added_texture_coordinate_idx++;
					}
					else
					{
						// Store index of the new (common) texture coordinate
						added_texture_coordinate_indices[std::make_pair(k, i)] = edge_texture_coordinate_idx_3;
					}
				}
			}
			else
			{
				// New vertex has already been created for this edge
				
				// Get index of the new edge vertex
				edge_vertex_3 = added_edge_vertices.find(std::make_pair(i, k))->second;

				if (_has_texture_coordinates)
				{
					// Get index of the new texture coordinate
					edge_texture_coordinate_idx_3 = added_texture_coordinate_indices.find(std::make_pair(i, k))->second;
				}
			}
			
			// Construct sub-faces (modifies the current face, but it will not be needed anymore anyway)
			if (_has_texture_coordinates)
			{
				_topology.setFace(face_idx, i, edge_vertex_1, edge_vertex_3, l, edge_texture_coordinate_idx_1, edge_texture_coordinate_idx_3);
				_topology.setNextFace(edge_vertex_1, j, edge_vertex_2, edge_texture_coordinate_idx_1, m, edge_texture_coordinate_idx_2);
				_topology.setNextFace(edge_vertex_1, edge_vertex_2, edge_vertex_3, edge_texture_coordinate_idx_1, edge_texture_coordinate_idx_2, edge_texture_coordinate_idx_3);
				_topology.setNextFace(edge_vertex_3, edge_vertex_2, k, edge_texture_coordinate_idx_3, edge_texture_coordinate_idx_2, n);
			}
			else
			{
				_topology.setFace(face_idx, i, edge_vertex_1, edge_vertex_3);
				_topology.setNextFace(edge_vertex_1, j, edge_vertex_2);
				_topology.setNextFace(edge_vertex_1, edge_vertex_2, edge_vertex_3);
				_topology.setNextFace(edge_vertex_3, edge_vertex_2, k);
			}
		}

		added_edge_vertices.clear();
		added_texture_coordinate_indices.clear();

		// Remove redundant vertex entries
		new_vertices.shed_cols(added_vertex_idx, new_vertices.n_cols-1);

		// Set homogeneous coordinates
		new_vertices.insert_rows(3, 1);
		new_vertices.row(3).ones();

		// Replace old vertices with the new ones
		_vertices.swap(new_vertices);
		new_vertices.clear();

		// The adjacency data is now invalid
		_topology.invalidateAdjacencyData();
	}

	// Recompute mesh attributes

	if (_has_face_normals)
	{
		_has_face_normals = false;
		computeFaceNormals();
	}

	if (_has_vertex_normals)
	{
		_has_vertex_normals = false;
		computeVertexNormals();
	}
	
	if (_has_texture_coordinates)
	{
		_has_vertex_texture_coordinate_indices = false;
		computeVertexTextureCoordinateIndices();
	}

	if (_has_vertex_tangents)
	{
		_has_vertex_tangents = false;
		computeTangentVectors();
	}

	_has_aabb = false;
}

void TriangleMesh::performCatmullClarkSubdivisions(imp_uint n_subdivisions,
												   imp_uint boundary_interpolation_mode /* = 0 */)
{
	assert(_is_homogenized);
	assert(!_has_vertex_data_3);
	assert(!_has_texture_coordinates);

	imp_uint n_vertices;
	imp_uint n_faces;
	imp_uint vertex_idx;
	imp_uint face_idx;
	
	arma::Mat<imp_float> new_vertices;
	imp_uint approx_n_new_vertices;
	imp_uint added_vertex_idx;
	imp_uint added_texture_coordinate_idx;

	std::list<imp_uint> adjacent_face_list;
	std::list<imp_uint> neighbour_vertex_list;
	std::list<imp_uint>::const_iterator neighbour_vertex, adjacent_face;
	imp_uint n_neighbour_vertices;
	imp_uint n_adjacent_faces;
	imp_float neighbour_weight, self_weight;
	arma::Col<imp_float> average_face_point(3);
	arma::Col<imp_float> average_edge_midpoint(3);
	
	imp_uint i, j, k;
	imp_uint l, m, n;
	imp_uint l_adjacent, m_adjacent, n_adjacent;
	imp_uint adjacent_faces[3];
	imp_uint face_vertex, adjacent_face_vertex;
	imp_uint edge_vertex_1, edge_vertex_2, edge_vertex_3;
	std::map<std::pair<imp_uint, imp_uint>, imp_uint> added_edge_vertices;

	assert(boundary_interpolation_mode == 0 || boundary_interpolation_mode == 1);
	
	const imp_float boundary_neighbour_weight = (boundary_interpolation_mode)? 0.0f : 1.0f/8.0f;
	const imp_float boundary_self_weight	  = (boundary_interpolation_mode)? 1.0f : 6.0f/8.0f;

	for (imp_uint subdivision_i = 0; subdivision_i < n_subdivisions; subdivision_i++)
	{
		n_vertices = getNumberOfVertices();
		n_faces = getNumberOfFaces();
		
		// Set upper bound on the number of new vertices to create.
		// A vertex is created for each face and each edge, and there are (3 - avg_neighbours/2)*n_faces
		// edges, where avg_neighbours (between 0 and 3) is the average number of faces adjacent to a
		// face in the mesh. Setting avg_neighbours = 1 and adding a safety padding of 3 should
		// be a safe limit.
		approx_n_new_vertices = n_faces + 5*n_faces/2 + 3;

		// Temporary storage of modified and new vertices
		new_vertices.set_size(3, n_vertices + approx_n_new_vertices);



		// Counter for keeping track of next available vertex index
		added_vertex_idx = n_vertices;
		
		// Counter for keeping track of next available texture coordinate index
		added_texture_coordinate_idx = getNumberOfTextureCoordinates();

		// Make sure the mesh has adjacency data
		_topology.generateAdjacencyData(n_vertices);

		// Loop through all faces
		for (face_idx = 0; face_idx < n_faces; face_idx++)
		{
			// Create new vertex as the average of the three face vertices

			_topology.getFace(face_idx, i, j, k);

			self_weight = 1.0f/3.0f;

			new_vertices(0, added_vertex_idx) = self_weight*(_vertices(0, i) + _vertices(0, j) + _vertices(0, k));
			new_vertices(1, added_vertex_idx) = self_weight*(_vertices(1, i) + _vertices(1, j) + _vertices(1, k));
			new_vertices(2, added_vertex_idx) = self_weight*(_vertices(2, i) + _vertices(2, j) + _vertices(2, k));

			added_vertex_idx++;
		}

		// Create modified versions of existing vertices
		for (vertex_idx = 0; vertex_idx < n_vertices; vertex_idx++)
		{
			// Get a list of faces that are touching - and vertices that are connected to - the current vertex
			_topology.findConnectedFacesAndVertices(vertex_idx, adjacent_face_list, neighbour_vertex_list);

			n_adjacent_faces = static_cast<imp_uint>(adjacent_face_list.size());
			n_neighbour_vertices = static_cast<imp_uint>(neighbour_vertex_list.size());

			if (n_neighbour_vertices == n_adjacent_faces)
			{
				// The vertex is in the interior of the surface
				
				const arma::Col<imp_float>& vertex_point = _vertices.col(vertex_idx).subvec(0, 2);
				const imp_float& norm = 1.0f/n_neighbour_vertices;

				// Compute the average of the surrounding face points created by the previous loop,
				// as well as the average midpoint of all edges connected to the vertex

				average_face_point.zeros();
				average_edge_midpoint.zeros();

				adjacent_face = adjacent_face_list.begin();
				neighbour_vertex = neighbour_vertex_list.begin();

				while (adjacent_face != adjacent_face_list.end())
				{
					average_face_point += new_vertices.col(n_vertices + *adjacent_face);
					average_edge_midpoint += (vertex_point + _vertices.col(*neighbour_vertex).subvec(0, 2))*0.5f;
				
					adjacent_face++;
					neighbour_vertex++;
				}

				average_face_point *= norm;
				average_edge_midpoint *= norm;

				// Compute new vertex position
				new_vertices.col(vertex_idx) = (average_face_point + 2.0f*average_edge_midpoint + (n_neighbour_vertices - 3.0f)*vertex_point)*norm;
			}
			else
			{
				// The vertex is on the boundary of the surface
	
				new_vertices(0, vertex_idx) = boundary_self_weight*_vertices(0, vertex_idx) + boundary_neighbour_weight*(_vertices(0, neighbour_vertex_list.front()) + _vertices(0, neighbour_vertex_list.back()));
				new_vertices(1, vertex_idx) = boundary_self_weight*_vertices(1, vertex_idx) + boundary_neighbour_weight*(_vertices(1, neighbour_vertex_list.front()) + _vertices(1, neighbour_vertex_list.back()));
				new_vertices(2, vertex_idx) = boundary_self_weight*_vertices(2, vertex_idx) + boundary_neighbour_weight*(_vertices(2, neighbour_vertex_list.front()) + _vertices(2, neighbour_vertex_list.back()));
			}
		
			// Empty list of adjacent faces and neighbour vertices
			adjacent_face_list.clear();
			neighbour_vertex_list.clear();
		}

		// Make room for the required number of new faces
		_topology.reserveAdditionalFaces(5*n_faces);

		// Loop through all faces
		for (face_idx = 0; face_idx < n_faces; face_idx++)
		{
			// Get indices of face vertices
			_topology.getFace(face_idx, i, j, k);

			// Get texture coordinate indices if they exist
			if (_topology.hasTextureCoordinateIndices())
				_topology.getTextureCoordinateIndices(face_idx, l, m, n);

			// Find faces adjacent to the current face
			_topology.findAdjacentFaces(face_idx, adjacent_faces);

			// Index of the face vertex for this face added by the first face loop
			face_vertex = n_vertices + face_idx;

			// For each of the three face edges, add a new vertex if it
			// hasn't already been created in an earlier loop iteration

			// Edge i <-> j
			if (adjacent_faces[0] > face_idx)
			{
				// New vertex has not been created for this edge
			
				edge_vertex_1 = added_vertex_idx;

				self_weight = 0.25f; // 1/4
				neighbour_weight = 0.25f; // 1/4

				adjacent_face_vertex = n_vertices + adjacent_faces[0];

				new_vertices(0, edge_vertex_1) = self_weight*(_vertices(0, i) + _vertices(0, j)) + neighbour_weight*(new_vertices(0, face_vertex) + new_vertices(0, adjacent_face_vertex));
				new_vertices(1, edge_vertex_1) = self_weight*(_vertices(1, i) + _vertices(1, j)) + neighbour_weight*(new_vertices(1, face_vertex) + new_vertices(1, adjacent_face_vertex));
				new_vertices(2, edge_vertex_1) = self_weight*(_vertices(2, i) + _vertices(2, j)) + neighbour_weight*(new_vertices(2, face_vertex) + new_vertices(2, adjacent_face_vertex));
		
				 // Store index of the new vertex
				added_edge_vertices[std::make_pair(i, j)] = edge_vertex_1;

				added_vertex_idx++;
			}
			else if (adjacent_faces[0] < face_idx)
			{
				// New vertex has already been created for this edge

				// Get index of the new edge vertex
				edge_vertex_1 = added_edge_vertices.find(std::make_pair(j, i))->second;
			}
			else
			{
				// Boundary edge
			
				edge_vertex_1 = added_vertex_idx;

				self_weight = 0.5f;

				new_vertices(0, edge_vertex_1) = self_weight*(_vertices(0, i) + _vertices(0, j));
				new_vertices(1, edge_vertex_1) = self_weight*(_vertices(1, i) + _vertices(1, j));
				new_vertices(2, edge_vertex_1) = self_weight*(_vertices(2, i) + _vertices(2, j));
			
				 // Store index of the new vertex
				added_edge_vertices[std::make_pair(i, j)] = edge_vertex_1;

				added_vertex_idx++;
			}

			// Edge j <-> k
			if (adjacent_faces[1] > face_idx)
			{
				// New vertex has not been created for this edge
			
				edge_vertex_2 = added_vertex_idx;

				self_weight = 0.25f; // 1/4
				neighbour_weight = 0.25f; // 1/4

				adjacent_face_vertex = n_vertices + adjacent_faces[1];

				new_vertices(0, edge_vertex_2) = self_weight*(_vertices(0, j) + _vertices(0, k)) + neighbour_weight*(new_vertices(0, face_vertex) + new_vertices(0, adjacent_face_vertex));
				new_vertices(1, edge_vertex_2) = self_weight*(_vertices(1, j) + _vertices(1, k)) + neighbour_weight*(new_vertices(1, face_vertex) + new_vertices(1, adjacent_face_vertex));
				new_vertices(2, edge_vertex_2) = self_weight*(_vertices(2, j) + _vertices(2, k)) + neighbour_weight*(new_vertices(2, face_vertex) + new_vertices(2, adjacent_face_vertex));
		
				 // Store index of the new vertex
				added_edge_vertices[std::make_pair(j, k)] = edge_vertex_2;

				added_vertex_idx++;
			}
			else if (adjacent_faces[1] < face_idx)
			{
				// New vertex has already been created for this edge
				
				// Get index of the new edge vertex
				edge_vertex_2 = added_edge_vertices.find(std::make_pair(k, j))->second;
			}
			else
			{
				// Boundary edge
			
				edge_vertex_2 = added_vertex_idx;

				self_weight = 0.5f;

				new_vertices(0, edge_vertex_2) = self_weight*(_vertices(0, j) + _vertices(0, k));
				new_vertices(1, edge_vertex_2) = self_weight*(_vertices(1, j) + _vertices(1, k));
				new_vertices(2, edge_vertex_2) = self_weight*(_vertices(2, j) + _vertices(2, k));
			
				 // Store index of the new vertex
				added_edge_vertices[std::make_pair(j, k)] = edge_vertex_2;

				added_vertex_idx++;
			}

			// Edge k <-> i
			if (adjacent_faces[2] > face_idx)
			{
				// New vertex has not been created for this edge
			
				edge_vertex_3 = added_vertex_idx;

				self_weight = 0.25f; // 1/4
				neighbour_weight = 0.25f; // 1/4

				adjacent_face_vertex = n_vertices + adjacent_faces[2];

				new_vertices(0, edge_vertex_3) = self_weight*(_vertices(0, k) + _vertices(0, i)) + neighbour_weight*(new_vertices(0, face_vertex) + new_vertices(0, adjacent_face_vertex));
				new_vertices(1, edge_vertex_3) = self_weight*(_vertices(1, k) + _vertices(1, i)) + neighbour_weight*(new_vertices(1, face_vertex) + new_vertices(1, adjacent_face_vertex));
				new_vertices(2, edge_vertex_3) = self_weight*(_vertices(2, k) + _vertices(2, i)) + neighbour_weight*(new_vertices(2, face_vertex) + new_vertices(2, adjacent_face_vertex));
		
				 // Store index of the new vertex
				added_edge_vertices[std::make_pair(k, i)] = edge_vertex_3;

				added_vertex_idx++;
			}
			else if (adjacent_faces[2] < face_idx)
			{
				// New vertex has already been created for this edge
				
				// Get index of the new edge vertex
				edge_vertex_3 = added_edge_vertices.find(std::make_pair(i, k))->second;
			}
			else
			{
				// Boundary edge
			
				edge_vertex_3 = added_vertex_idx;

				self_weight = 0.5f;

				new_vertices(0, edge_vertex_3) = self_weight*(_vertices(0, k) + _vertices(0, i));
				new_vertices(1, edge_vertex_3) = self_weight*(_vertices(1, k) + _vertices(1, i));
				new_vertices(2, edge_vertex_3) = self_weight*(_vertices(2, k) + _vertices(2, i));
			
				 // Store index of the new vertex
				added_edge_vertices[std::make_pair(k, i)] = edge_vertex_3;

				added_vertex_idx++;
			}

			// Construct sub-faces (modifies the current face, but it will not be needed anymore anyway)
			_topology.setFace(face_idx, face_vertex, i, edge_vertex_1);
			_topology.setNextFace(face_vertex, edge_vertex_1, j);
			_topology.setNextFace(face_vertex, j, edge_vertex_2);
			_topology.setNextFace(face_vertex, edge_vertex_2, k);
			_topology.setNextFace(face_vertex, k, edge_vertex_3);
			_topology.setNextFace(face_vertex, edge_vertex_3, i);
		}

		added_edge_vertices.clear();

		// Remove redundant vertex entries
		new_vertices.shed_cols(added_vertex_idx, new_vertices.n_cols-1);

		// Set homogeneous coordinates
		new_vertices.insert_rows(3, 1);
		new_vertices.row(3).ones();

		// Replace old vertices with the new ones
		_vertices.swap(new_vertices);
		new_vertices.clear();

		// The adjacency data is now invalid
		_topology.invalidateAdjacencyData();
	}

	// Recompute mesh attributes

	if (_has_face_normals)
	{
		_has_face_normals = false;
		computeFaceNormals();
	}

	if (_has_vertex_normals)
	{
		_has_vertex_normals = false;
		computeVertexNormals();
	}
	
	if (_has_texture_coordinates)
	{
		_has_vertex_texture_coordinate_indices = false;
		computeVertexTextureCoordinateIndices();
	}
	
	if (_has_vertex_tangents)
	{
		_has_vertex_tangents = false;
		computeTangentVectors();
	}

	_has_aabb = false;
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

void TriangleMesh::_clip(imp_uint component, imp_float limit, imp_int sign)
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
		if (_has_texture_coordinates)
			_topology.getFace(idx, i, j, k, l, m, n);
		else
			_topology.getFace(idx, i, j, k);
        
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
					if (_has_texture_coordinates)
					{
						_topology.setFace(idx,
										  inside_idx_1, inside_idx_2, last_vertex + 1,
										  texture_inside_idx_1, texture_inside_idx_2, last_texture_coord + 1);

						_topology.addFace(last_vertex + 1, inside_idx_2, last_vertex + 2,
										  last_texture_coord + 1, texture_inside_idx_2, last_texture_coord + 2);
						
						last_texture_coord += 2;
					}
					else
					{
						_topology.setFace(idx,
										  inside_idx_1, inside_idx_2, last_vertex + 1);

						_topology.addFace(last_vertex + 1, inside_idx_2, last_vertex + 2);
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
					if (_has_texture_coordinates)
					{
						_topology.setFace(idx,
										  inside_idx_1, last_vertex + 1, last_vertex + 2,
										  texture_inside_idx_1, last_texture_coord + 1, last_texture_coord + 2);

						last_texture_coord += 2;
					}
					else
					{
						_topology.setFace(idx,
										  inside_idx_1, last_vertex + 1, last_vertex + 2);
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
	if (_has_vertex_tangents)
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
		_topology.getFace(idx, i, j, k);

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
	_topology.getFace(0, i, j, k);
    x = _vertices(0, i), y = _vertices(1, i), z = _vertices(2, i);

    _aabb.lower_corner.moveTo(x, y, z);
    _aabb.upper_corner.moveTo(x, y, z);

    for (imp_uint face_idx = 1; face_idx < n_faces; face_idx++)
    {
		_topology.getFace(face_idx, i, j, k);

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

void TriangleMesh::computeFaceNormals(bool force /* = false */)
{
    if (_has_face_normals && !force)
		return;
    
    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k;

    _face_normals = arma::Mat<imp_float>(3, n_faces, arma::fill::zeros);

    for (imp_uint idx = 0; idx < n_faces; idx++)
    {
		_topology.getFace(idx, i, j, k);

        const Vector& normal = Triangle::areaVector(Point(_vertices(0, i), _vertices(1, i), _vertices(2, i)),
                                                    Point(_vertices(0, j), _vertices(1, j), _vertices(2, j)),
                                                    Point(_vertices(0, k), _vertices(1, k), _vertices(2, k)));

        _face_normals(0, idx) = normal.x;
        _face_normals(1, idx) = normal.y;
        _face_normals(2, idx) = normal.z;
    }
    
    _has_face_normals = true;
}

void TriangleMesh::computeVertexNormals(bool force /* = false */)
{
    if (_has_vertex_normals && !force)
		return;

	assert(_has_face_normals);

    _vertex_normals = arma::Mat<imp_float>(3, getNumberOfVertices(), arma::fill::zeros);
    
    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k;

    for (imp_uint idx = 0; idx < n_faces; idx++)
    {
		_topology.getFace(idx, i, j, k);

		_vertex_normals.col(i) += _face_normals.col(idx);
		_vertex_normals.col(j) += _face_normals.col(idx);
		_vertex_normals.col(k) += _face_normals.col(idx);
    }

	_vertex_normals = arma::normalise(_vertex_normals);
    
    _has_vertex_normals = true;
}

void TriangleMesh::computeTangentVectors(bool force /* = false */)
{
    if (_has_vertex_tangents && !force)
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
		_topology.getFace(idx, i, j, k, l, m, n);

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

void TriangleMesh::computeVertexTextureCoordinateIndices(bool force /* = false */)
{
	if (_has_vertex_texture_coordinate_indices && !force)
		return;
	
	assert(_has_texture_coordinates);
	
	imp_uint i, j, k;
	imp_uint l, m, n;
	imp_uint invalid_index = getNumberOfTextureCoordinates() + 1;

	_vertex_texture_coordinate_indices.clear();
	_vertex_texture_coordinate_indices.resize(getNumberOfVertices(), invalid_index);
	
    for (imp_uint face_idx = 0; face_idx < getNumberOfFaces(); face_idx++)
    {
		_topology.getFace(face_idx, i, j, k, m, n, l);

		if (_vertex_texture_coordinate_indices[i] == invalid_index)
			_vertex_texture_coordinate_indices[i] = l;

		if (_vertex_texture_coordinate_indices[j] == invalid_index)
			_vertex_texture_coordinate_indices[j] = m;

		if (_vertex_texture_coordinate_indices[k] == invalid_index)
			_vertex_texture_coordinate_indices[k] = n;
	}

	_has_vertex_texture_coordinate_indices = true;
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
	imp_uint i, j, k;
	
	_topology.getFace(face_idx, i, j, k);

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
	
	imp_uint i, j, k;

	_topology.getFace(intersection_data.face_id, i, j, k);

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

void TriangleMesh::displaceVertexInNormalDirection(imp_uint idx, imp_float displacement_distance)
{
	assert(_has_vertex_normals);

	_vertices(0, idx) += _vertex_normals(0, idx)*displacement_distance;
	_vertices(1, idx) += _vertex_normals(1, idx)*displacement_distance;
	_vertices(2, idx) += _vertex_normals(2, idx)*displacement_distance;
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
	
	imp_uint i, j, k;

	_topology.getFace(face_idx, i, j, k);

    vertex_normals[0].setComponents(_vertex_normals(0, i), _vertex_normals(1, i), _vertex_normals(2, i));
    vertex_normals[1].setComponents(_vertex_normals(0, j), _vertex_normals(1, j), _vertex_normals(2, j));
    vertex_normals[2].setComponents(_vertex_normals(0, k), _vertex_normals(1, k), _vertex_normals(2, k));
}

void TriangleMesh::getVertexTangentsForFace(imp_uint face_idx, Vector tangents[3], Vector bitangents[3]) const
{
	assert(_has_vertex_tangents);
	
	imp_uint i, j, k;

	_topology.getFace(face_idx, i, j, k);

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

const Geometry2D::Point& TriangleMesh::getVertexTextureCoordinate(imp_uint idx) const
{
	assert(_has_texture_coordinates);
	assert(_has_vertex_texture_coordinate_indices);

	return _texture_coordinates[_vertex_texture_coordinate_indices[idx]];
}

Triangle TriangleMesh::getFace(imp_uint face_idx) const
{
	imp_uint i, j, k;

	_topology.getFace(face_idx, i, j, k);

    return Triangle(getVertex(i), getVertex(j), getVertex(k));
}

void TriangleMesh::getFaceVertices(imp_uint face_idx, Point vertices[3]) const
{
	imp_uint i, j, k;

	_topology.getFace(face_idx, i, j, k);

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
	
	imp_uint i, j, k;
	
	_topology.getFace(face_idx, i, j, k);

	return Vector(alpha*_vertex_normals(0, i) + beta*_vertex_normals(0, j) + gamma*_vertex_normals(0, k),
				  alpha*_vertex_normals(1, i) + beta*_vertex_normals(1, j) + gamma*_vertex_normals(1, k),
				  alpha*_vertex_normals(2, i) + beta*_vertex_normals(2, j) + gamma*_vertex_normals(2, k)).getNormalized();
}

Vector TriangleMesh::getInterpolatedVertexNormal(const MeshIntersectionData& intersection_data) const
{
    assert(_has_vertex_normals);

	imp_uint i, j, k;

	_topology.getFace(intersection_data.face_id, i, j, k);
	
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
	
	imp_uint i, j, k;

	_topology.getFace(face_idx, i, j, k);

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

	imp_uint l, m, n;

	_topology.getTextureCoordinateIndices(face_idx, l, m, n);

	texture_coordinates[0] = _texture_coordinates[l];
	texture_coordinates[1] = _texture_coordinates[m];
	texture_coordinates[2] = _texture_coordinates[n];
}

Geometry2D::Point TriangleMesh::getInterpolatedTextureCoordinates(imp_uint face_idx, imp_float alpha, imp_float beta, imp_float gamma) const
{
    assert(_has_texture_coordinates);
	
	imp_uint l, m, n;

	_topology.getTextureCoordinateIndices(face_idx, l, m, n);

	return Point2(_texture_coordinates[l] + (_texture_coordinates[m] - _texture_coordinates[l])*beta + (_texture_coordinates[n] - _texture_coordinates[l])*gamma);
}

Geometry2D::Point TriangleMesh::getInterpolatedTextureCoordinates(const MeshIntersectionData& intersection_data) const
{
    assert(_has_texture_coordinates);

	imp_uint l, m, n;

	_topology.getTextureCoordinateIndices(intersection_data.face_id, l, m, n);
		
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
	imp_uint i, j, k;

	_topology.getFace(face_idx, i, j, k);

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

	imp_uint i, j, k;

	_topology.getFace(face_idx, i, j, k);

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
    return _topology.getNumberOfFaces();
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
    return _topology.getFacesString();
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
    imp_uint i, j, k;

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
			_topology.getFace(idx, i, j, k);

            outfile << "f " << i+1 << "//" << i+1 << " " <<
                               j+1 << "//" << j+1 << " " <<
                               k+1 << "//" << k+1 << std::endl;
        }
    }
    else
    {

        for (idx = 0; idx < n_faces; idx++)
        {
			_topology.getFace(idx, i, j, k);

            outfile << "f " << i+1 << " " << 
                               j+1 << " " <<
                               k+1 << std::endl;
        }
    }

    outfile.close();
}

} // Geometry3D
} // Impact
