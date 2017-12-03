#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Box.hpp"
#include "Sphere.hpp"
#include "AxisAlignedBox.hpp"
#include "Triangle3.hpp"
#include "BoundingVolumeHierarchy.hpp"
#include "Camera.hpp"
#include "Point2.hpp"
#include "Triangle2.hpp"
#include "BoundingAreaHierarchy.hpp"
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
	typedef Geometry2D::Point Point2;
	typedef Geometry2D::Triangle Triangle2;
    typedef Geometry2D::AxisAlignedRectangle AxisAlignedRectangle;
    typedef Geometry2D::AABRContainer AABRContainer;
	typedef Geometry2D::BoundingAreaHierarchy BoundingAreaHierarchy;
    typedef std::vector<std::string> string_vec;
    typedef std::vector<imp_int> int_vec;

    const imp_float _eps_impact_factor = 1e-7f;
    const imp_float _eps_coordinates = -1e-10f;

    arma::Mat<imp_float> _vertices;
    arma::Mat<imp_uint> _faces;
    arma::Mat<imp_float> _normals;
    arma::Mat<imp_float> _vertex_data_3;

    AxisAlignedBox _aabb;
    BoundingVolumeHierarchy _bounding_volume_hierarchy;
    BoundingAreaHierarchy _bounding_area_hierarchy;
    
    bool _is_homogenized;
    bool _has_normals;
    bool _has_aabb;
	bool _has_vertex_data_3;

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

    void _clip(imp_uint component, imp_float limit, int sign);

    bool _addIntersectionVertices(imp_uint i, imp_uint j, imp_uint k,
                                  imp_float origin[], imp_float other_1[], imp_float other_2[],
                                  imp_uint component_1, imp_uint component_2, imp_uint component_3,
                                  imp_float limit);

public:
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

	void initializeVertexData3();
	void setVertexData3(imp_uint idx,
						imp_float data_0,
						imp_float data_1,
						imp_float data_2);

    void addVertex(imp_float x, imp_float y, imp_float z);
    void addVertex(const Point& vertex);
    void addFace(imp_uint i, imp_uint j, imp_uint k);

    void removeVertex(imp_uint idx);
    void removeFace(imp_uint idx);

    void splitFaces(imp_uint n_times);

    void clipNearPlaneAt(imp_float z_near);
    void clipLeftPlane();
    void clipRightPlane();
    void clipTopPlane();
    void clipBottomPlane();
    void clipNearPlane();
    void clipFarPlane();

    void clipNonNearPlanes();

    void removeBackwardFacingFaces();
	
    void computeAABB();
    void computeBoundingVolumeHierarchy();
    void computeBoundingAreaHierarchy(imp_float image_width,
									  imp_float image_height,
									  imp_float inverse_image_width_at_unit_distance_from_camera,
									  imp_float inverse_image_height_at_unit_distance_from_camera);
    
    void computeNormalVectors();
	void normalizeNormalVectors();

    void homogenizeVertices();

    imp_float evaluateRayIntersection(const Ray& ray) const;
    bool evaluateRayAABBIntersection(const Ray& ray) const;
    imp_float evaluateRayFaceIntersectionNonOptimized(const Ray& ray, imp_uint face_idx, imp_float& alpha, imp_float& beta, imp_float& gamma) const;
    imp_float evaluateRayFaceIntersection(const Ray& ray, imp_uint face_idx, imp_float& alpha, imp_float& beta, imp_float& gamma) const;
    imp_float evaluateRayFaceIntersectionDistanceOnly(const Ray& ray, imp_uint face_idx) const;

    TriangleMesh& applyTransformation(const LinearTransformation& transformation);
    TriangleMesh& applyTransformation(const AffineTransformation& transformation);
    TriangleMesh& applyTransformation(const ProjectiveTransformation& transformation);
    TriangleMesh& applyWindowingTransformation(const AffineTransformation& transformation);

    Point getVertex(imp_uint idx) const;
	Vector getVertexNormal(imp_uint idx) const;
	void getVertexData3(imp_uint idx,
						imp_float& data_0,
						imp_float& data_1,
						imp_float& data_2) const;
    Triangle getFace(imp_uint face_idx) const;
	void getFaceVertices(imp_uint face_idx, Point vertices[3]) const;
	void getFaceNormals(imp_uint face_idx, Vector normals[3]) const;
	void getFaceVertexData3(imp_uint face_idx,
							imp_float data_A[3],
							imp_float data_B[3],
							imp_float data_C[3]) const;
	void getFaceAttributes(imp_uint face_idx, Point vertices[3], Vector normals[3]) const;
    Point2 getProjectedVertex(imp_uint idx,
							  imp_float image_width,
							  imp_float image_height,
							  imp_float inverse_image_width_at_unit_distance_from_camera,
							  imp_float inverse_image_height_at_unit_distance_from_camera) const;
    Triangle2 getProjectedFace(imp_uint face_idx,
							   imp_float image_width,
							   imp_float image_height,
							   imp_float inverse_image_width_at_unit_distance_from_camera,
							   imp_float inverse_image_height_at_unit_distance_from_camera) const;
	
    std::vector<imp_uint> getIntersectedFaceIndices(const Ray& ray) const;
    std::vector<imp_uint> getIntersectedFaceIndices(const Point2& pixel_center) const;

    const AxisAlignedBox& getAABB() const;

    imp_uint getNumberOfVertices() const;
    imp_uint getNumberOfFaces() const;

    std::string getVerticesString() const;
    std::string get4SpaceVerticesString() const;
    std::string getFacesString() const;

    bool isHomogenized() const;
    bool hasNormals() const;
    bool hasAABB() const;

	bool allZAbove(imp_float z_low) const;
	bool isInsideParallelViewVolume() const;
	bool faceFacesOrigin(imp_uint face_idx) const;
	bool isOutsideViewFrustum(const Plane& lower_plane,
							  const Plane& upper_plane,
							  const Plane& left_plane,
							  const Plane& right_plane,
							  imp_float far_plane_distance) const;
	
    void saveAs(const std::string& filename) const;
};

} // Geometry3D
} // Impact
