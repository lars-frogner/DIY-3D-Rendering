#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Box.hpp"
#include "Sphere.hpp"
#include "AxisAlignedBox.hpp"
#include "Triangle3.hpp"
#include "BoundingVolumeHierarchy.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"
#include <armadillo>
#include <vector>
#include <string>

namespace Impact {
namespace Geometry3D {

class TriangleMesh {

protected:
    typedef std::vector<std::string> string_vec;
    typedef std::vector<imp_int> int_vec;

    const imp_float _eps_impact_factor = 1e-7f;
    const imp_float _eps_coordinates = -1e-10f;

    arma::Mat<imp_float> _vertices;
    arma::Mat<imp_uint> _faces;
    arma::Mat<imp_float> _normals;

    AxisAlignedBox _aabb;
    BoundingVolumeHierarchy _bounding_volume_hierarchy;
    
    bool _is_homogenized;
    bool _has_normals;

    static imp_float _getCoordinateFromObjString(const std::string& s);
    static imp_uint _getFaceIndexFromObjString(const std::string& s, imp_uint n_vertices);

    static void _getObjFileData(const std::string& filename,
                                string_vec& vertices,
                                string_vec& texture_coords,
                                string_vec& normals,
                                string_vec& param_space_vertices,
                                string_vec& faces,
                                string_vec& material_files,
                                string_vec& material_names,
                                int_vec& face_material_indices);

public:
    bool uses_omp = true;

    TriangleMesh();
    TriangleMesh(imp_uint n_vertices, imp_float vertex_array[],
                 imp_uint n_faces, imp_uint face_array[]);
    TriangleMesh(const TriangleMesh& mesh_1,
                 const TriangleMesh& mesh_2);
    
    static TriangleMesh file(const std::string& filename, string_vec& = string_vec(), string_vec& = string_vec());
    static TriangleMesh triangle(const Triangle& triangle_obj);
    static TriangleMesh box(const Box& box_obj);
    static TriangleMesh room(const Box& box_obj);
    static TriangleMesh sheet(const Point& origin,
                              const Vector& width_vector,
                              const Vector& height_vector);
    static TriangleMesh sphere(const Sphere& sphere_obj, imp_uint resolution);

    imp_uint getNumberOfVertices() const;
    imp_uint getNumberOfFaces() const;

    void addVertex(imp_float x, imp_float y, imp_float z);
    void addVertex(const Point& vertex);
    void addFace(imp_uint i, imp_uint j, imp_uint k);

    void removeVertex(imp_uint idx);
    void removeFace(imp_uint idx);

    void splitFaces(imp_uint n_times);
    
    void computeNormals();

    void applyTransformation(const LinearTransformation& transformation);
    void applyTransformation(const AffineTransformation& transformation);
    void applyTransformation(const ProjectiveTransformation& transformation);
    void applyWindowingTransformation(const AffineTransformation& transformation);

    void homogenizeVertices();

    void computeAABB();
    void computeBoundingVolumeHierarchy();

	void getVertexAttributes(imp_uint face_idx, Point vertices[3], Vector normals[3]) const;
    
    std::vector<imp_uint> getIntersectedFaceIndices(const Ray& ray) const;
    imp_float evaluateRayIntersection(const Ray& ray) const;
    
    bool evaluateRayAABBIntersection(const Ray& ray) const;
    imp_float evaluateRayFaceIntersectionNonOptimized(const Ray& ray, imp_uint face_idx, imp_float& alpha, imp_float& beta, imp_float& gamma) const;
    imp_float evaluateRayFaceIntersection(const Ray& ray, imp_uint face_idx, imp_float& alpha, imp_float& beta, imp_float& gamma) const;
    imp_float evaluateRayFaceIntersectionDistanceOnly(const Ray& ray, imp_uint face_idx) const;

    Point getVertex(imp_uint idx) const;
    Triangle getFace(imp_uint idx) const;

    const AxisAlignedBox& getAABB() const;

    std::string getVerticesString() const;
    std::string get4SpaceVerticesString() const;
    std::string getFacesString() const;

    void saveAs(const std::string& filename) const;

    bool isHomogenized() const;
    bool hasNormals() const;
};

} // Geometry3D
} // Impact
