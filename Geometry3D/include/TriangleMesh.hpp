#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Box.hpp"
#include "Sphere.hpp"
#include "AxisAlignedBox.hpp"
#include "Triangle3.hpp"
#include "MeshTopology.hpp"
#include "BoundingVolumeHierarchy.hpp"
#include "Camera.hpp"
#include "Point2.hpp"
#include "Vector2.hpp"
#include "Triangle2.hpp"
#include "BoundingAreaHierarchy.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"
#include <armadillo>
#include <utility>
#include <vector>
#include <map>
#include <list>
#include <string>

namespace Impact {
namespace Geometry3D {

class TriangleMesh {

private:
	typedef Geometry2D::Point Point2;
	typedef Geometry2D::Vector Vector2;
	typedef Geometry2D::Triangle Triangle2;
    typedef Geometry2D::AxisAlignedRectangle AxisAlignedRectangle;
    typedef Geometry2D::AABRContainer AABRContainer;
	typedef Geometry2D::BoundingAreaHierarchy BoundingAreaHierarchy;
    typedef std::vector<std::string> string_vec;
    typedef std::vector<imp_int> int_vec;

protected:

    const imp_float _eps_impact_factor = 1e-7f;
    const imp_float _eps_coordinates = -1e-10f;

    arma::Mat<imp_float> _vertices;
    arma::Mat<imp_float> _vertex_normals;
    arma::Mat<imp_float> _face_normals;
	std::vector<Point2> _texture_coordinates;
	arma::Mat<imp_float> _vertex_tangents;
    arma::Mat<imp_float> _vertex_data_3;

	MeshTopology _topology;

    AxisAlignedBox _aabb;
    BoundingVolumeHierarchy _bounding_volume_hierarchy;
    BoundingAreaHierarchy _bounding_area_hierarchy;
    
    bool _is_homogenized;
    bool _has_vertex_normals;
    bool _has_face_normals;
	bool _has_texture_coordinates;
	bool _has_vertex_tangents;
	bool _has_vertex_data_3;
    bool _has_aabb;

    static imp_float _getCoordinateFromObjString(const std::string& s);
    static imp_uint _getFaceIndexFromObjString(const std::string& s, imp_uint n_vertices);
	static imp_uint _getFaceTextureIndexFromObjString(const std::string& s, imp_uint n_texture_coordinates);
    static void _getObjFileData(const std::string& filename,
                                string_vec& vertices,
                                string_vec& texture_coords,
                                string_vec& normals,
                                string_vec& param_space_vertices,
                                string_vec& faces,
                                string_vec& material_files,
                                string_vec& material_names,
                                int_vec& face_material_indices);

    void _clip(imp_uint component, imp_float limit, imp_int sign);

    bool _addIntersectionVertices(imp_uint i, imp_uint j, imp_uint k,
                                  imp_uint l, imp_uint m, imp_uint n,
                                  imp_float origin[], imp_float other_1[], imp_float other_2[],
                                  imp_uint component_1, imp_uint component_2, imp_uint component_3,
                                  imp_float limit);

public:
    TriangleMesh();
    
    static TriangleMesh file(const std::string& filename, string_vec& = string_vec(), string_vec& = string_vec());
    static TriangleMesh triangle(const Triangle& triangle_obj);
    static TriangleMesh box(const Box& box_obj);
    static TriangleMesh manifoldBox(const Box& box_obj);
    static TriangleMesh room(const Box& box_obj);
    static TriangleMesh sheet(const Point& center,
							  const Vector& normal,
							  const Vector& width_vector,
							  imp_float height);
    static TriangleMesh twoSidedSheet(const Point& center,
									  const Vector& normal,
									  const Vector& width_vector,
									  imp_float height);
    static TriangleMesh sphere(const Sphere& sphere_obj, imp_uint resolution, imp_uint texture_mapping_mode = 0);
    static TriangleMesh twoSidedSphere(const Sphere& sphere_obj, imp_uint resolution);

	void initializeVertexData3();
	void setVertexData3(imp_uint idx,
						imp_float data_0,
						imp_float data_1,
						imp_float data_2);

    void addVertex(imp_float x, imp_float y, imp_float z);
    void addVertex(const Point& vertex);
    void addFace(imp_uint i, imp_uint j, imp_uint k);
    void addFace(imp_uint i, imp_uint j, imp_uint k,
				 imp_uint l, imp_uint m, imp_uint n);

    void removeVertex(imp_uint idx);
    void removeFace(imp_uint idx);

	void performLoopSubdivisions(imp_uint n_subdivisions);

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
    
    void computeFaceNormals();
    void computeVertexNormals();
	void computeTangentVectors();

    void homogenizeVertices();

    imp_float evaluateRayIntersection(const Ray& ray, MeshIntersectionData& intersection_data) const;
    bool evaluateRayAABBIntersection(const Ray& ray) const;
    imp_float evaluateRayFaceIntersectionNonOptimized(const Ray& ray, imp_uint face_idx, imp_float& alpha, imp_float& beta, imp_float& gamma) const;
    imp_float evaluateRayFaceIntersection(const Ray& ray, MeshIntersectionData& intersection_data) const;

    TriangleMesh& applyTransformation(const LinearTransformation& transformation);
    TriangleMesh& applyTransformation(const AffineTransformation& transformation);
    TriangleMesh& applyTransformation(const ProjectiveTransformation& transformation);
    TriangleMesh& applyWindowingTransformation(const AffineTransformation& transformation);

    Point getVertex(imp_uint idx) const;

	Vector getVertexNormal(imp_uint idx) const;

	void getVertexTangents(imp_uint idx, Vector& tangent, Vector& bitangent) const;

	void getVertexNormalsForFace(imp_uint face_idx, Vector vertex_normals[3]) const;

	void getVertexTangentsForFace(imp_uint face_idx, Vector tangents[3], Vector bitangents[3]) const;

	void getVertexData3(imp_uint idx,
						imp_float& data_0,
						imp_float& data_1,
						imp_float& data_2) const;

    Triangle getFace(imp_uint face_idx) const;

	void getFaceVertices(imp_uint face_idx, Point vertices[3]) const;
	
	Vector getFaceNormal(imp_uint face_idx) const;

	Vector getInterpolatedVertexNormal(imp_uint face_idx, imp_float alpha, imp_float beta, imp_float gamma) const;
	Vector getInterpolatedVertexNormal(const MeshIntersectionData& intersection_data) const;
	
	void getInterpolatedVertexTangents(imp_uint face_idx,
									   imp_float alpha, imp_float beta, imp_float gamma,
									   Vector& tangent, Vector& bitangent) const;
	void getInterpolatedVertexTangents(const MeshIntersectionData& intersection_data,
									   Vector& tangent, Vector& bitangent) const;

	void getVertexData3ForFace(imp_uint face_idx,
							   imp_float data_A[3],
							   imp_float data_B[3],
							   imp_float data_C[3]) const;
	
	void getTextureCoordinates(imp_uint face_idx, Point2 texture_coordinates[3]) const;

	Point2 getInterpolatedTextureCoordinates(imp_uint face_idx, imp_float alpha, imp_float beta, imp_float gamma) const;
	Point2 getInterpolatedTextureCoordinates(const MeshIntersectionData& intersection_data) const;

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
	
    std::vector<imp_uint> getPotentiallyIntersectedFaceIndices(const Ray& ray) const;
    void addPotentiallyIntersectedFaceIndices(const Point2& pixel_center, std::list<imp_uint>& face_indices) const;

    const AxisAlignedBox& getAABB() const;
	Point getCentroid() const;

    imp_uint getNumberOfVertices() const;
    imp_uint getNumberOfFaces() const;
    imp_uint getNumberOfTextureCoordinates() const;

    std::string getVerticesString() const;
    std::string get4SpaceVerticesString() const;
    std::string getFacesString() const;

    bool isHomogenized() const;
	bool hasFaceNormals() const;
	bool hasVertexNormals() const;
	bool hasTextureCoordinates() const;
	bool hasVertexTangents() const;
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

struct MeshIntersectionData
{
	imp_uint mesh_id;
	imp_uint face_id;
	imp_float alpha, beta, gamma;
};

} // Geometry3D
} // Impact
