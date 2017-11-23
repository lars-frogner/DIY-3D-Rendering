#pragma once
#include "precision.hpp"
#include "TriangleMesh.hpp"
#include "Color.hpp"
#include "Material.hpp"
#include "BlinnPhongMaterial.hpp"
#include "Light.hpp"
#include "Image.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Box.hpp"
#include "Sphere.hpp"
#include "AxisAlignedBox.hpp"
#include "Triangle3.hpp"
#include "Ray.hpp"
#include "BoundingVolumeHierarchy.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"
#include "Point2.hpp"
#include "Triangle2.hpp"
#include "AxisAlignedRectangle.hpp"
#include "BoundingAreaHierarchy.hpp"
#include <armadillo>
#include <vector>
#include <string>

namespace Impact {
namespace Rendering3D {

// Forward declaration of Scene class
class Scene;

class RenderableTriangleMesh : public Geometry3D::TriangleMesh {

private:
	typedef Geometry3D::TriangleMesh TriangleMesh;
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::Box Box;
	typedef Geometry3D::Sphere Sphere;
	typedef Geometry3D::AxisAlignedBox AxisAlignedBox;
	typedef Geometry3D::Triangle Triangle;
	typedef Geometry3D::Ray Ray;
	typedef Geometry3D::BoundingVolumeHierarchy BoundingVolumeHierarchy;
	typedef Geometry3D::LinearTransformation LinearTransformation;
	typedef Geometry3D::AffineTransformation AffineTransformation;
	typedef Geometry3D::ProjectiveTransformation ProjectiveTransformation;
    typedef Geometry2D::Point Point2;
    typedef Geometry2D::Triangle Triangle2;
    typedef Geometry2D::AxisAlignedRectangle AxisAlignedRectangle;
    typedef Geometry2D::AABRContainer AABRContainer;
    typedef Geometry2D::BoundingAreaHierarchy BoundingAreaHierarchy;
    typedef std::vector<std::string> string_vec;
    typedef std::vector<imp_int> int_vec;

    Material* _material = NULL;
    std::vector<Color> _colors;

    BoundingAreaHierarchy _bounding_area_hierarchy;
    
    bool _has_material;
    
    static std::vector<BlinnPhongMaterial> RenderableTriangleMesh::_getMtlFileData(const std::string& filename);

    void _clip(imp_uint component, imp_float limit, int sign);

    void _addIntersectionVertices(imp_uint i, imp_uint j, imp_uint k,
                                  imp_float origin[], imp_float other_1[], imp_float other_2[],
                                  imp_uint component_1, imp_uint component_2, imp_uint component_3,
                                  imp_float limit);

public:
	bool is_dynamic = false;
	
    bool uses_direct_lighting = true;
	bool casts_shadows = false;
	
    bool render_faces = true;
    bool render_edges = false;

    bool remove_hidden_faces = true;
    bool perform_clipping = true;
	
	bool is_visible = true;

    RenderableTriangleMesh();
    RenderableTriangleMesh(imp_uint n_vertices, imp_float vertex_array[],
						   imp_uint n_faces, imp_uint face_array[]);
    RenderableTriangleMesh(const RenderableTriangleMesh& mesh_1,
						   const RenderableTriangleMesh& mesh_2);
    RenderableTriangleMesh(const TriangleMesh& other);
    RenderableTriangleMesh(const RenderableTriangleMesh& other);
    ~RenderableTriangleMesh();
	RenderableTriangleMesh operator=(const RenderableTriangleMesh& other);
    
    static RenderableTriangleMesh file(const std::string& filename);
    static RenderableTriangleMesh triangle(const Triangle& triangle_obj);
    static RenderableTriangleMesh box(const Box& box_obj);
    static RenderableTriangleMesh room(const Box& box_obj);
    static RenderableTriangleMesh sheet(const Point& origin,
									    const Vector& width_vector,
										const Vector& height_vector);
    static RenderableTriangleMesh sphere(const Sphere& sphere_obj, imp_uint resolution);

    void setMaterial(const BlinnPhongMaterial& material);
    void setMaterialReplaceOld(const BlinnPhongMaterial& material);
    RenderableTriangleMesh withMaterial(const BlinnPhongMaterial& material);
    RenderableTriangleMesh withMaterialReplaceOld(const BlinnPhongMaterial& material);

    void shadeVerticesDirect(const Scene& scene);

    Triangle2 getProjectedFace(const Scene& scene, imp_uint face_idx) const;
    void getVertexAttributes(imp_uint face_idx, Point vertices[3], Vector normals[3], Material*& material) const;

    void computeBoundingAreaHierarchy(const Scene& scene);
    
    std::vector<imp_uint> getIntersectedFaceIndices(const Point2& pixel_center) const;

    bool sampleRadianceFromFace(const Scene& scene,
                                const Ray& ray,
                                imp_uint face_idx,
                                Radiance& pixel_radiance,
                                imp_float& closest_distance) const;

	bool allZAbove(imp_float z_low) const;
	bool isInsideParallelViewVolume() const;

    void clipNearPlaneAt(imp_float z_near);
    void clipLeftPlane();
    void clipRightPlane();
    void clipTopPlane();
    void clipBottomPlane();
    void clipNearPlane();
    void clipFarPlane();

    void clipNonNearPlanes();

    void removeBackwardFacingFaces();

	void drawEdges(Image& image, float luminance) const;
	void drawFaces(Image& image) const;
	void drawFaces(Image& image, Color color) const;

    void saveAs(const std::string& filename, const std::string& material_filename = std::string()) const;

    bool hasMaterial() const;
};

} // Rendering3D
} // Impact
