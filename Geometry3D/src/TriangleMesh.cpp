#include "TriangleMesh.hpp"
#include "string_util.hpp"
#include <omp.h>
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
      _has_normals(false),
	  uses_omp(true) {}

TriangleMesh::TriangleMesh(imp_uint n_vertices, imp_float vertex_array[],
                           imp_uint n_faces, imp_uint face_array[])
    : _vertices(4, n_vertices, arma::fill::ones),
      _faces(face_array, 3, n_faces),
      _is_homogenized(true),
      _has_normals(false),
	  uses_omp(true)
{
    _vertices.rows(0, 2) = arma::Mat<imp_float>(vertex_array, 3, n_vertices);
}

TriangleMesh::TriangleMesh(const TriangleMesh& mesh_1,
                           const TriangleMesh& mesh_2)
    : _vertices(arma::join_cols(mesh_1._vertices, mesh_2._vertices)),
      _faces(arma::join_cols(mesh_1._faces, mesh_2._faces + mesh_2.getNumberOfVertices())),
      _is_homogenized(mesh_1._is_homogenized && mesh_2._is_homogenized),
      _has_normals(false),
	  uses_omp(true) {}

TriangleMesh TriangleMesh::file(const std::string& filename, string_vec& material_files, string_vec& material_names)
{
    string_vec vertices;
    string_vec texture_coords;
    string_vec normals;
    string_vec param_space_vertices;
    string_vec faces;
    int_vec face_material_indices;

    _getObjFileData(filename,
                    vertices,
                    texture_coords,
                    normals,
                    param_space_vertices,
                    faces,
                    material_files,
                    material_names,
                    face_material_indices);

    imp_uint n_vertices = static_cast<imp_uint>(vertices.size());
    imp_uint n_texture_coords = static_cast<imp_uint>(texture_coords.size());
    imp_uint n_normals = static_cast<imp_uint>(normals.size());
    imp_uint n_param_space_vertices = static_cast<imp_uint>(param_space_vertices.size());
    imp_uint n_faces = static_cast<imp_uint>(faces.size());
    imp_uint idx;
    imp_uint n_values;
    string_vec splitted;
    arma::Col<imp_uint> extra_face(3);

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

    // Normals
    
    mesh._has_normals = n_normals > 0;

    if (mesh._has_normals)
        mesh._normals = arma::Mat<imp_float>(3, n_vertices);

    for (idx = 0; idx < n_normals; idx++)
    {
        splitted = string_util::split(normals[idx]);
        n_values = static_cast<imp_uint>(splitted.size());

        if (n_values == 3)
        {
            mesh._normals(0, idx) = _getCoordinateFromObjString(splitted[0]);
            mesh._normals(1, idx) = _getCoordinateFromObjString(splitted[1]);
            mesh._normals(2, idx) = _getCoordinateFromObjString(splitted[2]);
        }
        else
        {
            std::cerr << "Error: invalid number of values (" << n_values << ") for normal " << idx << " (" << normals[idx] << ") in " << filename << std::endl;
            throw;
        }
    }
    
    if (mesh._has_normals)
        arma::normalise(mesh._normals);

    // Faces

    mesh._faces = arma::Mat<imp_uint>(3, n_faces);

    imp_uint face_list_idx = 0;
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

            face_list_idx++;

            mesh._faces.insert_cols(face_list_idx, extra_face);
            face_list_idx++;
        }
        else
        {
            std::cerr << "Error: invalid number of values (" << n_values << ") for face " << idx << " (" << faces[idx] << ") in " << filename << std::endl;
            throw;
        }
    }



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

    triangle_mesh._faces = {{0}, {1}, {2}};

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

    return room_mesh;
}

TriangleMesh TriangleMesh::sheet(const Point& origin, const Vector& width_vector, const Vector& height_vector)
{
    TriangleMesh sheet_mesh;

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
    imp_uint i, j, n;

    imp_float dtheta = IMP_PI/(static_cast<imp_float>(n_lat) - 1);
    imp_float dphi = 2*IMP_PI/static_cast<imp_float>(n_lon);

    imp_float theta, phi;
    imp_float sin_theta;
    imp_uint offset, offset_prev;
    imp_uint current, right, above, above_right;
    
    sphere_mesh._normals = arma::Mat<imp_float>(3, n_vertices);
    sphere_mesh._vertices = arma::Mat<imp_float>(4, n_vertices);
    sphere_mesh._vertices.row(3).ones();

    // Top vertex
    sphere_mesh._normals(0, 0) = 0;
    sphere_mesh._normals(1, 0) = 0;
    sphere_mesh._normals(2, 0) = 1;
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

            sphere_mesh._normals(0, n) = sin_theta*cos(phi);
            sphere_mesh._normals(1, n) = sin_theta*sin(phi);
            sphere_mesh._normals(2, n) = cos(theta);

            sphere_mesh._vertices(0, n) = x0 + r*sphere_mesh._normals(0, n);
            sphere_mesh._vertices(1, n) = y0 + r*sphere_mesh._normals(1, n);
            sphere_mesh._vertices(2, n) = z0 + r*sphere_mesh._normals(2, n);

            n++;
        }
    }
    
    // Bottom vertex
    sphere_mesh._normals(0, n) = 0;
    sphere_mesh._normals(1, n) = 0;
    sphere_mesh._normals(2, n) = -1;
    sphere_mesh._vertices(0, n) = x0;
    sphere_mesh._vertices(1, n) = y0;
    sphere_mesh._vertices(2, n) = z0 - r;

    sphere_mesh._faces = arma::Mat<imp_uint>(3, n_faces);
    
    // Top cone
    n = 0;
    for (j = 1; j < n_lon; j++)
    {
        sphere_mesh._faces(0, n) = j;
        sphere_mesh._faces(1, n) = j + 1;
        sphere_mesh._faces(2, n) = 0;
        n++;
    }
    sphere_mesh._faces(0, n) = n_lon;
    sphere_mesh._faces(1, n) = 1;
    sphere_mesh._faces(2, n) = 0;
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
            n++;

            sphere_mesh._faces(0, n) = above_right;
            sphere_mesh._faces(1, n) = above;
            sphere_mesh._faces(2, n) = current;
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
        n++;

        sphere_mesh._faces(0, n) = above_right;
        sphere_mesh._faces(1, n) = above;
        sphere_mesh._faces(2, n) = current;
        n++;
    }

    // Bottom cone
    offset = 1 + (n_lat - 3)*n_lon;
    for (j = 0; j < n_lon-1; j++)
    {
        sphere_mesh._faces(0, n) = offset + j + 1;
        sphere_mesh._faces(1, n) = offset + j;
        sphere_mesh._faces(2, n) = n_vertices - 1;
        n++;
    }
    sphere_mesh._faces(0, n) = offset + n_lon;
    sphere_mesh._faces(1, n) = offset + n_lon - 1;
    sphere_mesh._faces(2, n) = n_vertices - 1;

    return sphere_mesh;
}

imp_uint TriangleMesh::getNumberOfVertices() const
{
    return static_cast<imp_uint>(_vertices.n_cols);
}

imp_uint TriangleMesh::getNumberOfFaces() const
{
    return static_cast<imp_uint>(_faces.n_cols);
}

void TriangleMesh::addVertex(imp_float x, imp_float y, imp_float z)
{
    _vertices.insert_cols(_vertices.n_cols, arma::Col<imp_float>({x, y, z, 1}));
    _has_normals = false;
}

void TriangleMesh::addVertex(const Point& vertex)
{
    addVertex(vertex.x, vertex.y, vertex.z);
}

void TriangleMesh::addFace(imp_uint i, imp_uint j, imp_uint k)
{
    _faces.insert_cols(_faces.n_cols, arma::Col<imp_uint>({i, j, k}));
    _has_normals = false;
}

void TriangleMesh::removeVertex(imp_uint idx)
{
    arma::uvec occurences = arma::unique(arma::find(_faces == idx)/3);
    arma::uvec::const_iterator iter = occurences.end();
    while (iter != occurences.begin())
    {
        --iter;
        _faces.shed_col(*iter);
    }

    _vertices.shed_col(idx);

    occurences = arma::find(_faces > idx);
    for (iter = occurences.begin(); iter != occurences.end(); ++iter)
    {
        _faces(*iter) += 1;
    }

    _has_normals = false;
}

void TriangleMesh::removeFace(imp_uint idx)
{
    _faces.shed_col(idx);
    _has_normals = false;
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

void TriangleMesh::computeNormals()
{
    if (_has_normals) return;

    _normals = arma::Mat<imp_float>(3, getNumberOfVertices(), arma::fill::zeros);
    
    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k;

    for (imp_uint idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        const Vector& normal = Triangle::areaVector(Point(_vertices(0, i), _vertices(1, i), _vertices(2, i)),
                                                    Point(_vertices(0, j), _vertices(1, j), _vertices(2, j)),
                                                    Point(_vertices(0, k), _vertices(1, k), _vertices(2, k)));

        _normals(0, i) += normal.x;
        _normals(1, i) += normal.y;
        _normals(2, i) += normal.z;

        _normals(0, j) += normal.x;
        _normals(1, j) += normal.y;
        _normals(2, j) += normal.z;

        _normals(0, k) += normal.x;
        _normals(1, k) += normal.y;
        _normals(2, k) += normal.z;
    }

    _normals = arma::normalise(_normals);
    
    _has_normals = true;
}

void TriangleMesh::applyTransformation(const LinearTransformation& transformation)
{
    _vertices.rows(0, 2) = transformation.getMatrix().submat(0, 0, 2, 2)*_vertices.rows(0, 2);

    if (_has_normals)
        _normals = transformation.getNormalTransformMatrix()*_normals;
}

void TriangleMesh::applyTransformation(const AffineTransformation& transformation)
{
    _vertices = transformation.getMatrix()*_vertices;

    if (_has_normals)
        _normals = transformation.getNormalTransformMatrix()*_normals;
}

void TriangleMesh::applyTransformation(const ProjectiveTransformation& transformation)
{
    _vertices = transformation.getMatrix()*_vertices;
    _is_homogenized = false;
    _has_normals = false;
}

void TriangleMesh::applyWindowingTransformation(const AffineTransformation& transformation)
{
    assert(_is_homogenized);
    _vertices.rows(0, 1) = transformation.getMatrix().submat(0, 0, 1, 3)*_vertices;
    _has_normals = false;
}

void TriangleMesh::homogenizeVertices()
{
    const arma::Row<imp_float>& norm_vals = 1/_vertices.row(3);
    _vertices.each_row(arma::uvec({0, 1, 2})) %= norm_vals;
    _vertices.row(3).ones();

    _is_homogenized = true;
    _has_normals = false;

    return;
}

void TriangleMesh::getVertexAttributes(imp_uint face_idx, Point vertices[3], Vector normals[3]) const
{
    assert(_has_normals);

    imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

    vertices[0].moveTo(_vertices(0, i), _vertices(1, i), _vertices(2, i));
    vertices[1].moveTo(_vertices(0, j), _vertices(1, j), _vertices(2, j));
    vertices[2].moveTo(_vertices(0, k), _vertices(1, k), _vertices(2, k));

    normals[0].setComponents(_normals(0, i), _normals(1, i), _normals(2, i));
    normals[1].setComponents(_normals(0, j), _normals(1, j), _normals(2, j));
    normals[2].setComponents(_normals(0, k), _normals(1, k), _normals(2, k));
}

void TriangleMesh::computeAABB()
{
    _aabb.lower_corner = Point::max();
    _aabb.upper_corner = Point::min();
    
    imp_uint n_faces = getNumberOfFaces();
    imp_uint i, j, k;
    imp_float x, y, z;

    for (imp_uint face_idx = 0; face_idx < n_faces; face_idx++)
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
}

void TriangleMesh::computeBoundingVolumeHierarchy()
{
    imp_uint n_triangles = getNumberOfFaces();
    std::vector< AABBContainer > objects(n_triangles);

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

bool TriangleMesh::evaluateRayAABBIntersection(const Ray& ray) const
{
    return _aabb.evaluateRayIntersection(ray) < IMP_FLOAT_INF;
}

std::vector<imp_uint> TriangleMesh::getIntersectedFaceIndices(const Ray& ray) const
{
    return _bounding_volume_hierarchy.getIntersectedObjectIDs(ray);
}

imp_float TriangleMesh::evaluateRayIntersection(const Ray& ray) const
{
    return _bounding_volume_hierarchy.evaluateRayIntersection(*this, ray);
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

imp_float TriangleMesh::evaluateRayFaceIntersection(const Ray& ray, imp_uint face_idx, imp_float& alpha, imp_float& beta, imp_float& gamma) const
{
    // Bootleneck for ray tracing

    imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

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

    beta = (dx*qx + dy*qy + dz*qz)*iipf;

    if (beta < _eps_coordinates || beta > 1 - _eps_coordinates)
        return IMP_FLOAT_INF;

    imp_float rx = dy*ABz - dz*ABy, ry = dz*ABx - dx*ABz, rz = dx*ABy - dy*ABx;
    gamma = (rdx*rx + rdy*ry + rdz*rz)*iipf;

    if (gamma < _eps_coordinates || (alpha = 1 - (beta + gamma)) < _eps_coordinates)
        return IMP_FLOAT_INF;

    imp_float distance = (ACx*rx + ACy*ry + ACz*rz)*iipf;

    if (distance <= 0)
        return IMP_FLOAT_INF;

    return distance;
}

imp_float TriangleMesh::evaluateRayFaceIntersectionDistanceOnly(const Ray& ray, imp_uint face_idx) const
{
    // Bootleneck for ray tracing

    imp_uint i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

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

    imp_float beta = (dx*qx + dy*qy + dz*qz)*iipf;

    if (beta < _eps_coordinates || beta > (1 - _eps_coordinates))
        return IMP_FLOAT_INF;

    imp_float rx = dy*ABz - dz*ABy, ry = dz*ABx - dx*ABz, rz = dx*ABy - dy*ABx;
    imp_float gamma = (rdx*rx + rdy*ry + rdz*rz)*iipf;

    if (gamma < _eps_coordinates || beta + gamma > (1 - _eps_coordinates))
        return IMP_FLOAT_INF;

    imp_float distance = (ACx*rx + ACy*ry + ACz*rz)*iipf;

    if (distance <= 0)
        return IMP_FLOAT_INF;

    return distance;
}

Point TriangleMesh::getVertex(imp_uint idx) const
{
    return Point(_vertices(0, idx), _vertices(1, idx), _vertices(2, idx));
}

Triangle TriangleMesh::getFace(imp_uint idx) const
{
    return Triangle(getVertex(_faces(0, idx)), getVertex(_faces(1, idx)), getVertex(_faces(2, idx)));
}

const AxisAlignedBox& TriangleMesh::getAABB() const
{
    return _aabb;
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

bool TriangleMesh::isHomogenized() const
{
    return _is_homogenized;
}

bool TriangleMesh::hasNormals() const
{
    return _has_normals;
}

} // Geometry3D
} // Impact
